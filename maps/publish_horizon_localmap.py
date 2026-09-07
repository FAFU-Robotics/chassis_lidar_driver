#!/usr/bin/env python3
"""Accumulate Airy horizon-band points in the odom frame using wheel TF.

Inserts a wall-height slice at odom pose so RViz can show a local map
while you drive. Does not write .mm/.simplemap.
"""
from __future__ import annotations

import math
import struct
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener


def _xyz_i(msg: PointCloud2):
    fmap = {f.name: f for f in msg.fields}
    ox, oy, oz = fmap["x"].offset, fmap["y"].offset, fmap["z"].offset
    oi = fmap["intensity"].offset if "intensity" in fmap else None
    step = msg.point_step
    n = msg.width * msg.height
    data = msg.data
    xs, ys, zs, inten = [], [], [], []
    for i in range(n):
        base = i * step
        x = struct.unpack_from("<f", data, base + ox)[0]
        if not math.isfinite(x):
            continue
        y = struct.unpack_from("<f", data, base + oy)[0]
        z = struct.unpack_from("<f", data, base + oz)[0]
        if not (math.isfinite(y) and math.isfinite(z)):
            continue
        xs.append(x)
        ys.append(y)
        zs.append(z)
        if oi is not None:
            raw = struct.unpack_from("<f", data, base + oi)[0]
            inten.append(float(raw) if math.isfinite(raw) else 0.0)
        else:
            inten.append(0.0)
    if not xs:
        return None
    return np.column_stack(
        (
            np.asarray(xs, np.float32),
            np.asarray(ys, np.float32),
            np.asarray(zs, np.float32),
            np.asarray(inten, np.float32),
        )
    )


def _quat_to_R(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def _cloud(header, pts: np.ndarray) -> PointCloud2:
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(pts.shape[0])
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * msg.width
    msg.is_dense = True
    msg.data = pts.astype(np.float32, copy=False).tobytes()
    return msg


class HorizonLocalMap(Node):
    def __init__(self) -> None:
        super().__init__("horizon_localmap")
        self._z_min = -0.20
        self._z_max = 2.50
        self._r_min = 0.40
        self._r_max = 15.0
        self._voxel = 0.08
        self._vox: dict[tuple[int, int, int], np.ndarray] = {}
        self._buf = Buffer()
        self._tfl = TransformListener(self._buf, self)
        pub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(PointCloud2, "/horizon_localmap", pub_qos)
        self._v = 0.0
        self._w = 0.0
        self._yaw = 0.0
        self._x = 0.0
        self._y = 0.0
        self._yaw_hist: list[tuple[float, float]] = []
        self._straight_m = 0.0
        self._last_xy: tuple[float, float] | None = None
        self.create_subscription(
            PointCloud2, "/rslidar_points", self._on_cloud, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/wheel_odom", self._on_odom, 20)
        self.create_timer(0.4, self._flush)
        self._dirty = False
        self._n_in = 0
        self._n_skip = 0
        self._cleared = False
        self.create_timer(0.25, self._publish_empty_once)
        self.get_logger().info(
            "localmap in odom: insert only while |v|>=0.06, |w|<0.08, "
            "yaw stable, and >=0.35 m straight since last turn. "
            "One parked snapshot at start; then only straight driving."
        )

    def _publish_empty_once(self) -> None:
        if self._cleared:
            return
        self._cleared = True
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = "odom"
        self._pub.publish(_cloud(h, np.zeros((0, 4), np.float32)))
        self.get_logger().info("published empty localmap (RViz should clear)")

    def _on_odom(self, msg: Odometry) -> None:
        self._v = float(msg.twist.twist.linear.x)
        self._w = float(msg.twist.twist.angular.z)
        p = msg.pose.pose.position
        self._x, self._y = float(p.x), float(p.y)
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        if abs(self._w) > 0.08:
            self._straight_m = 0.0
            self._last_xy = (self._x, self._y)
        elif self._last_xy is None:
            self._last_xy = (self._x, self._y)
        else:
            dx = self._x - self._last_xy[0]
            dy = self._y - self._last_xy[1]
            self._straight_m += math.hypot(dx, dy)
            self._last_xy = (self._x, self._y)
        self._yaw = yaw

    def _yaw_stable(self) -> bool:
        now = time.monotonic()
        self._yaw_hist.append((now, self._yaw))
        self._yaw_hist = [(t, y) for t, y in self._yaw_hist if now - t <= 0.4]
        if len(self._yaw_hist) < 3:
            return False
        y0 = self._yaw_hist[0][1]
        span = max(
            abs(math.atan2(math.sin(y - y0), math.cos(y - y0)))
            for _, y in self._yaw_hist
        )
        return span < math.radians(2.5)

    def _on_cloud(self, msg: PointCloud2) -> None:
        moving = abs(self._v) >= 0.06
        turning = abs(self._w) > 0.08
        stable = self._yaw_stable()
        # One snapshot while parked so RViz is not empty after LiveScan is off.
        # After that, only straight driving adds points.
        origin_shot = (self._n_in == 0) and (not moving) and (not turning) and stable
        driving = moving and (not turning) and (self._straight_m >= 0.35) and stable
        if not origin_shot and not driving:
            self._n_skip += 1
            return
        arr = _xyz_i(msg)
        if arr is None:
            return
        try:
            tf = self._buf.lookup_transform(
                "odom",
                msg.header.frame_id or "rslidar",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            return
        R = _quat_to_R(tf.transform.rotation)
        t = np.array(
            [
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
            ],
            dtype=np.float32,
        )
        xyz = arr[:, :3] @ R.T + t
        # Range from the vehicle, not the odom origin — otherwise the map
        # stops growing once you drive past ~r_max meters from the start.
        rxy = np.hypot(xyz[:, 0] - t[0], xyz[:, 1] - t[1])
        m = (
            (xyz[:, 2] >= self._z_min)
            & (xyz[:, 2] <= self._z_max)
            & (rxy >= self._r_min)
            & (rxy <= self._r_max)
        )
        xyz = xyz[m]
        inten = arr[m, 3]
        if xyz.shape[0] < 20:
            return
        inv = 1.0 / self._voxel
        ijk = np.floor(xyz * inv).astype(np.int32)
        uid = (
            ijk[:, 0].astype(np.int64) * 1_000_003
            + ijk[:, 1].astype(np.int64) * 1_003
            + ijk[:, 2].astype(np.int64)
        )
        _, idx = np.unique(uid, return_index=True)
        for i in idx:
            key = (int(ijk[i, 0]), int(ijk[i, 1]), int(ijk[i, 2]))
            self._vox[key] = np.array(
                [xyz[i, 0], xyz[i, 1], xyz[i, 2], inten[i]], dtype=np.float32
            )
        self._dirty = True
        self._n_in += 1

    def _flush(self) -> None:
        if not self._dirty or not self._vox:
            return
        self._dirty = False
        pts = np.stack(list(self._vox.values()), axis=0)
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = "odom"
        self._pub.publish(_cloud(h, pts))
        if self._n_in % 10 == 0:
            self.get_logger().info(
                f"localmap voxels={pts.shape[0]} frames={self._n_in} "
                f"skip={self._n_skip} pose=({self._x:.1f},{self._y:.1f})"
            )


def main() -> None:
    rclpy.init()
    node = HorizonLocalMap()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
