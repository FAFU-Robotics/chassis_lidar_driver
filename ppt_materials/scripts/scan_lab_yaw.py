#!/usr/bin/python3
"""Slow in-place yaw scan for PPT denser lab cloud. Uses existing :9100 teleop.

v=0, |w|<=0.12, pulse 0.30s. Abort on stale MOLA pose / low quality / close obstacle.
Does not change MOLA, TF, or nav_demo planners.
"""
from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

ROOT = Path("/home/fafu_robot/Desktop/chassis_lidar_drivers")
sys.path.insert(0, str(ROOT / "nav_demo"))

os.environ.setdefault("ROS_HOME", "/tmp/ros_ppt_spin")
os.makedirs(os.environ["ROS_HOME"] + "/log", exist_ok=True)

W_CMD = 0.12
PULSE_S = 0.30
PERIOD_S = 0.20
YAW_TARGET_RAD = math.radians(250.0)
TIMEOUT_S = 32.0
POSE_STALE_S = 0.70
Q_MIN = 0.42
NEAR_ABORT_M = 0.36


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_q(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main() -> int:
    from official_mods import ensure_package
    from run_first_live import can_control_mode, halt_chassis

    ensure_package()
    from bunker_mini.teleop_tcp import DEFAULT_PORT, DEFAULT_TOKEN, TeleopTcpClient

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Float32

    mode = can_control_mode()
    print(f"PREFLIGHT control_mode={mode}", flush=True)
    if mode != 1:
        print("FAIL control_mode!=1, not moving")
        return 8

    qos_tl = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )
    qos_be = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )

    box = {"pose": None, "t_pose": 0.0, "q": None, "near": 99.0}

    def on_pose(msg: Odometry) -> None:
        p = msg.pose.pose.position
        yaw = yaw_from_q(msg.pose.pose.orientation)
        box["pose"] = (float(p.x), float(p.y), float(yaw))
        box["t_pose"] = time.monotonic()

    def on_q(msg: Float32) -> None:
        box["q"] = float(msg.data)

    def on_cloud(msg: PointCloud2) -> None:
        n = int(msg.width) * int(msg.height)
        if n <= 0:
            return
        off = {f.name: f.offset for f in msg.fields}
        step = int(msg.point_step)
        data = bytes(msg.data)
        import struct

        near = 99.0
        # subsample for speed
        stride = max(1, n // 4000)
        for i in range(0, n, stride):
            o = i * step
            x, y, z = struct.unpack_from("<fff", data, o + off["x"])
            if not (x == x and y == y and z == z):
                continue
            # sensor: y up, z forward, x left
            xb, yb, zb = z, x, y + 0.365
            if zb < 0.12 or zb > 1.20:
                continue
            r = math.hypot(xb, yb)
            if r < 0.30:
                continue  # robot body / self returns
            if r < near:
                near = r
        box["near"] = near

    rclpy.init()
    node = Node("ppt_lab_spin")
    node.create_subscription(Odometry, "/lidar_odometry/pose", on_pose, qos_tl)
    node.create_subscription(Float32, "/lidar_odometry/pose_quality", on_q, qos_tl)
    node.create_subscription(PointCloud2, "/rslidar_points", on_cloud, qos_be)

    t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if box["pose"] is not None and box["q"] is not None:
            break
    if box["pose"] is None:
        print("FAIL no pose")
        node.destroy_node()
        rclpy.shutdown()
        return 2

    sx, sy, syaw = box["pose"]
    print(
        f"START pose=({sx:.3f},{sy:.3f},{math.degrees(syaw):.1f}) q={box['q']} near={box['near']:.2f}",
        flush=True,
    )
    if box["q"] is not None and box["q"] < Q_MIN:
        print("FAIL quality too low")
        node.destroy_node()
        rclpy.shutdown()
        return 3
    if box["near"] < NEAR_ABORT_M:
        print(f"FAIL obstacle too close {box['near']:.2f}m")
        node.destroy_node()
        rclpy.shutdown()
        return 4

    cli = TeleopTcpClient("127.0.0.1", DEFAULT_PORT, DEFAULT_TOKEN)
    try:
        cli.hello()
        cli.stick(0.0, 0.0)
    except Exception as exc:
        print(f"FAIL :9100 {exc}")
        node.destroy_node()
        rclpy.shutdown()
        return 6

    n_pulse = 0
    reason = "timeout"
    last_pulse = 0.0
    try:
        t_run = time.monotonic()
        while time.monotonic() - t_run < TIMEOUT_S:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            age = now - box["t_pose"]
            pose = box["pose"]
            if pose is None or age > POSE_STALE_S:
                reason = f"pose_stale age={age:.3f}"
                break
            q = box["q"]
            if q is not None and q < Q_MIN:
                reason = f"quality {q:.3f}"
                break
            if box["near"] < NEAR_ABORT_M:
                reason = f"near {box['near']:.2f}m"
                break
            dyaw = abs(wrap(pose[2] - syaw))
            if dyaw >= YAW_TARGET_RAD:
                reason = f"yaw_done {math.degrees(dyaw):.1f}deg"
                break
            if now - last_pulse >= PERIOD_S:
                cli.cmd(
                    {
                        "action": "move",
                        "v": 0.0,
                        "w": float(W_CMD),
                        "duration": PULSE_S,
                        "bypassGuard": True,
                        "ts": int(time.time() * 1000),
                    },
                    timeout=0.8,
                )
                n_pulse += 1
                last_pulse = now
                if n_pulse % 8 == 1:
                    print(
                        f"PULSE n={n_pulse} yaw={math.degrees(pose[2]):.1f} "
                        f"d={math.degrees(dyaw):.1f} q={q} near={box['near']:.2f}",
                        flush=True,
                    )
        else:
            reason = "timeout"
    except KeyboardInterrupt:
        reason = "KeyboardInterrupt"
    except Exception as exc:
        reason = f"exception {exc}"
    finally:
        halt_chassis(cli, reason)
        time.sleep(0.3)
        try:
            cli.close()
        except Exception:
            pass

    pose = box["pose"] or (sx, sy, syaw)
    dyaw = abs(wrap(pose[2] - syaw))
    print(
        f"END pose=({pose[0]:.3f},{pose[1]:.3f},{math.degrees(pose[2]):.1f}) "
        f"dyaw={math.degrees(dyaw):.1f} n_pulse={n_pulse} reason={reason!r} q={box['q']}",
        flush=True,
    )
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if "yaw_done" in reason else 7


if __name__ == "__main__":
    sys.exit(main())
