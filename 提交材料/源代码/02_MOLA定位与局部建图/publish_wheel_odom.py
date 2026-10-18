#!/usr/bin/env python3
"""Listen-only SocketCAN odometry → /wheel_odom. Stdlib AF_CAN, no python-can.

Bunker Mini 0x311 wheel counters often freeze while driving (RC, or in-place
spin). Always fall back to integrating 0x221 v/w whenever 0x311 is stale,
same idea as bunker_mini.controller live odometer. Do not publish TF.
"""
from __future__ import annotations

import math
import os
import socket
import struct
import sys
import time

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster

_CAN_FRAME = struct.Struct("=IB3x8s")
ID_ODOM = 0x311
ID_MOTION = 0x221
WHEELBASE_M = 0.5


def _channel_score(ch: str, window_s: float = 0.4) -> int:
    """How many 0x221/0x311 frames appear on *ch* in a short sniff."""
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((ch,))
        sock.settimeout(0.15)
        n = 0
        t0 = time.monotonic()
        while time.monotonic() - t0 < window_s:
            try:
                raw = sock.recv(16)
            except socket.timeout:
                continue
            if len(raw) < 16:
                continue
            can_id = _CAN_FRAME.unpack(raw)[0] & 0x1FFFFFFF
            if can_id in (ID_ODOM, ID_MOTION):
                n += 1
        return n
    except OSError:
        return -1
    finally:
        sock.close()


def pick_bunker_can_channel() -> str:
    """USB CAN index swaps on reboot. Prefer the socket that actually carries
    Bunker 0x221/0x311. BUNKER_CAN_CHANNEL=can0|can1 forces that name."""
    forced = os.environ.get("BUNKER_CAN_CHANNEL", "").strip()
    if forced and forced not in ("auto",):
        n = _channel_score(forced)
        if n > 0:
            return forced
        # Forced port is silent: fall through and pick the live one.
    ranked = []
    for ch in ("can0", "can1"):
        n = _channel_score(ch)
        if n > 0:
            ranked.append((n, ch))
    if ranked:
        ranked.sort(reverse=True)
        return ranked[0][1]
    return forced or "can0"
# If 0x311 does not advance for this long, integrate 0x221 instead.
STALE_311_S = 0.15
# Reject a single-tick jump larger than this (wrap / drop).
MAX_WHEEL_STEP_M = 2.0


def _yaw_q(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class WheelOdom(Node):
    def __init__(self) -> None:
        super().__init__("bunker_wheel_odom")
        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(Odometry, "/wheel_odom", qos)
        self._tf = TransformBroadcaster(self)
        self.x = self.y = self.yaw = 0.0
        self._v = self._w = 0.0
        self._last_l = self._last_r = None
        self._last_311_move = 0.0
        self._last_221 = time.monotonic()
        self._src = "init"
        self._ticks = 0
        ch = pick_bunker_can_channel()
        self._sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._sock.bind((ch,))
        self._sock.setblocking(False)
        self.get_logger().info(
            f"listening {ch} 0x311+0x221 fuse → /wheel_odom (stale 0x311 → 0x221)"
        )
        self.create_timer(0.02, self._tick)

    def _integrate(self, d: float, dyaw: float) -> None:
        mid = self.yaw + dyaw / 2.0
        self.yaw += dyaw
        self.x += d * math.cos(mid)
        self.y += d * math.sin(mid)

    def _tick(self) -> None:
        moved_311 = False
        while True:
            try:
                raw = self._sock.recv(16)
            except BlockingIOError:
                break
            if len(raw) < 16:
                break
            can_id, dlc, data = _CAN_FRAME.unpack(raw)
            can_id &= 0x1FFFFFFF
            payload = data[:dlc]
            if can_id == ID_ODOM and dlc >= 8:
                left, right = struct.unpack(">ii", payload[:8])
                if self._last_l is None:
                    self._last_l, self._last_r = left, right
                else:
                    dl = (left - self._last_l) / 1000.0
                    dr = (right - self._last_r) / 1000.0
                    self._last_l, self._last_r = left, right
                    if abs(dl) < MAX_WHEEL_STEP_M and abs(dr) < MAX_WHEEL_STEP_M:
                        if abs(dl) > 1e-4 or abs(dr) > 1e-4:
                            # 0x311 often freezes then jumps; that catch-up
                            # delta is not real motion — skip it, keep 0x221.
                            thawed = (
                                time.monotonic() - self._last_311_move
                            ) > STALE_311_S
                            if not thawed:
                                self._integrate(
                                    (dl + dr) / 2.0, (dr - dl) / WHEELBASE_M
                                )
                                self._src = "0x311"
                                moved_311 = True
                            self._last_311_move = time.monotonic()
            elif can_id == ID_MOTION and dlc >= 4:
                lin, ang = struct.unpack(">hh", payload[:4])
                self._v = lin / 1000.0
                self._w = ang / 1000.0
                now = time.monotonic()
                dt = now - self._last_221
                self._last_221 = now
                stale = (now - self._last_311_move) > STALE_311_S
                if (not moved_311) and stale and 0.0 < dt < 0.2:
                    self._integrate(self._v * dt, self._w * dt)
                    if abs(self._v) > 1e-4 or abs(self._w) > 1e-4:
                        self._src = "0x221"
        self._publish()
        self._ticks += 1
        if self._ticks % 100 == 0:
            self.get_logger().info(
                f"{self._src} x={self.x:.3f} y={self.y:.3f} "
                f"yaw={math.degrees(self.yaw):.1f}° v={self._v:.3f} w={self._w:.3f}"
            )

    def _publish(self) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = _yaw_q(self.yaw)
        msg.twist.twist.linear.x = self._v
        msg.twist.twist.angular.z = self._w
        cov = [0.0] * 36
        cov[0] = 0.02
        cov[7] = 0.02
        cov[14] = 1.0
        cov[21] = 1.0
        cov[28] = 1.0
        cov[35] = 0.01
        msg.pose.covariance = cov
        self._pub.publish(msg)
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation = msg.pose.pose.orientation
        self._tf.sendTransform(tf)


def main() -> None:
    rclpy.init()
    node = WheelOdom()
    try:
        rclpy.spin(node)
    finally:
        node._sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
