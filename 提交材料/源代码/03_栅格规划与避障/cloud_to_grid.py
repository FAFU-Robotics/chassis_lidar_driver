#!/usr/bin/env python3
"""nav_demo: deskewed map cloud + MOLA pose → 2D OccupancyGrid. No chassis."""
from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _load_occupancy():
    """Load occupancy.py by file path so bunker_mini/__init__.py (agent) is not imported."""
    import importlib.util

    path = ROOT / "bunker_jetson" / "bunker_mini" / "occupancy.py"
    spec = importlib.util.spec_from_file_location("nav_demo_occupancy", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_occ = _load_occupancy()
PseudoGrid = _occ.OccupancyGrid
OCCUPIED = _occ.OCCUPIED

# OccupancyGrid.update uses vehicle (x=right, y=forward):
#   vx = dist * sin(az), vy = dist * cos(az)
# so azimuth = atan2(right, forward), 0° = forward.
# ROS base_link (REP-103): x = forward, y = left, z = up.
# From bunker_mini.rslidar_cloud.rslidar_to_vehicle (verified comment + code):
#   ROS base_link = (z_s, x_s, y_s + height)  i.e. x_fwd=sensor_z, y_left=sensor_x
#   agent vehicle right = -sensor_x = -base_y_left
#   agent vehicle forward = sensor_z = base_x_fwd
# Deskewed cloud is map frame → transform by pose (map→base_link), then:
#   x_right = -y_left, y_fwd = x_fwd.


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_to_rot(x: float, y: float, z: float, w: float):
    import numpy as np

    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def xyz_from_cloud(msg):
    import numpy as np

    n = int(msg.width) * int(msg.height)
    if n <= 0:
        return np.zeros((0, 3), np.float32)
    off = {f.name: f.offset for f in msg.fields}
    if not all(k in off for k in "xyz"):
        return np.zeros((0, 3), np.float32)
    dt = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": ["<f4", "<f4", "<f4"],
            "offsets": [off["x"], off["y"], off["z"]],
            "itemsize": int(msg.point_step),
        }
    )
    rec = np.frombuffer(bytes(msg.data), dtype=dt, count=n)
    xyz = np.column_stack((rec["x"], rec["y"], rec["z"]))
    return xyz[np.isfinite(xyz).all(axis=1)]


def map_points_to_base(xyz_map, t, R):
    """p_base = R^T (p_map - t). R rotates base_link → map."""
    import numpy as np

    if xyz_map.size == 0:
        return xyz_map.astype(np.float64, copy=False)
    rel = xyz_map.astype(np.float64, copy=False) - t.reshape(1, 3)
    return rel @ R


def sectors_from_base_xy(x_fwd, y_left, max_range: float, bin_deg: float = 2.0, min_range: float = 0.45):
    """ROS base_link XY → OccupancyGrid sectors (az_deg, dist)."""
    import numpy as np

    x_right = -y_left
    y_fwd = x_fwd
    dist = np.hypot(x_right, y_fwd)
    ok = (dist >= min_range) & (dist <= max_range)
    if not np.any(ok):
        return []
    az = np.degrees(np.arctan2(x_right[ok], y_fwd[ok]))
    d = dist[ok]
    nb = max(1, int(round(360.0 / bin_deg)))
    bins = np.floor((az + 180.0) / bin_deg).astype(np.int32)
    bins = np.clip(bins, 0, nb - 1)
    best: dict[int, float] = {}
    for b, di in zip(bins.tolist(), d.tolist()):
        prev = best.get(b)
        if prev is None or di < prev:
            best[b] = di
    out = []
    for b, di in best.items():
        az_mid = -180.0 + (b + 0.5) * bin_deg
        out.append((float(az_mid), float(di)))
    return out


def inflate_cells(occupied: set[tuple[int, int]], radius_cells: int) -> set[tuple[int, int]]:
    extra: set[tuple[int, int]] = set()
    for cx, cy in occupied:
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                extra.add((cx + dx, cy + dy))
    return extra


class CloudToGrid:
    def __init__(self, args: argparse.Namespace) -> None:
        import rclpy
        from nav_msgs.msg import OccupancyGrid as RosOccupancyGrid
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import PointCloud2

        self.args = args
        self._lock = threading.Lock()
        self._pose_xyyaw: tuple[float, float, float] | None = None
        self._pose_t = None
        self._pose_R = None
        self._last_valid = 0
        self._last_raw = 0
        self._last_print = 0.0
        self._cloud_source = "none"
        self._z_keep_frac = 0.0
        self._z_min_seen = 0.0
        self._z_max_seen = 0.0
        self._last_deskewed_mono = 0.0
        self._last_rslidar_mono = 0.0
        self.grid = PseudoGrid(
            resolution_m=args.resolution,
            max_range_m=args.range,
            ttl_s=args.ttl,
        )
        qos_tl = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        qos_be = QoSProfile(
            depth=8,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.node = Node("nav_demo_cloud_to_grid")
        self.node.create_subscription(Odometry, "/lidar_odometry/pose", self._on_pose, qos_tl)
        # Deskewed may be Transient Local or Volatile depending on MOLA env; subscribe both.
        self.node.create_subscription(
            PointCloud2, "/lidar_odometry/deskewed_scan_points", self._on_deskewed, qos_tl
        )
        self.node.create_subscription(
            PointCloud2, "/lidar_odometry/deskewed_scan_points", self._on_deskewed, qos_be
        )
        self.node.create_subscription(
            PointCloud2, "/rslidar_points", self._on_rslidar, qos_be
        )
        qos_rel_vol = QoSProfile(
            depth=8,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.node.create_subscription(
            PointCloud2, "/rslidar_points", self._on_rslidar, qos_rel_vol
        )
        self._pub = self.node.create_publisher(RosOccupancyGrid, "/nav_demo/obstacle_grid", qos_tl)
        self.node.create_timer(0.2, self._on_timer)
        self.node.get_logger().info(
            "nav_demo cloud_to_grid: /rslidar_points preferred for occupancy density; "
            "deskewed fallback; publish /nav_demo/obstacle_grid — no chassis, no stick, no CAN"
        )

    def _on_pose(self, msg) -> None:
        import numpy as np

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        t = np.array([p.x, p.y, p.z], dtype=np.float64)
        R = quat_to_rot(q.x, q.y, q.z, q.w)
        with self._lock:
            self._pose_xyyaw = (float(p.x), float(p.y), float(yaw))
            self._pose_t = t
            self._pose_R = R

    def _ingest(self, x_fwd, y_left, z_up, source: str) -> None:
        with self._lock:
            pose = self._pose_xyyaw
        if pose is None:
            return
        n = int(x_fwd.shape[0])
        self._last_raw = n
        if n == 0:
            self._last_valid = 0
            self._cloud_source = source
            self._z_keep_frac = 0.0
            return
        self._z_min_seen = float(z_up.min())
        self._z_max_seen = float(z_up.max())
        keep = (z_up >= self.args.z_min) & (z_up <= self.args.z_max)
        self._z_keep_frac = float(keep.mean())
        x_fwd, y_left = x_fwd[keep], y_left[keep]
        self._last_valid = int(x_fwd.shape[0])
        self._cloud_source = source
        if self._last_valid == 0:
            return
        sectors = sectors_from_base_xy(
            x_fwd, y_left, self.args.range, min_range=self.args.min_range
        )
        self.grid.update(pose[0], pose[1], math.degrees(pose[2]), sectors)

    def _on_deskewed(self, msg) -> None:
        # Occupancy uses denser vehicle-frame /rslidar_points when live.
        if time.monotonic() - self._last_rslidar_mono < 1.0:
            return
        with self._lock:
            t = self._pose_t
            R = self._pose_R
        if t is None or R is None:
            return
        xyz = xyz_from_cloud(msg)
        if xyz.shape[0] == 0:
            return
        base = map_points_to_base(xyz, t, R)
        # ROS base_link: x forward, y left, z up
        self._last_deskewed_mono = time.monotonic()
        self._ingest(base[:, 0], base[:, 1], base[:, 2], "deskewed")

    def _on_rslidar(self, msg) -> None:
        # Exact rslidar_to_vehicle mapping (sensor → ROS base_link):
        #   x_fwd = z_s, y_left = x_s, z_up = y_s + 0.365
        xyz = xyz_from_cloud(msg)
        if xyz.shape[0] == 0:
            return
        x_s, y_s, z_s = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        self._last_rslidar_mono = time.monotonic()
        self._ingest(z_s, x_s, y_s + 0.365, "rslidar")

    def _stats(self) -> tuple[int, int, set, set]:
        occ: set[tuple[int, int]] = set()
        for (cx, cy), val in self.grid.iter_cells():
            if val >= OCCUPIED:
                occ.add((cx, cy))
        radius = int(math.ceil(self.args.inflation / self.args.resolution))
        inf = inflate_cells(occ, radius)
        return len(occ), len(inf), occ, inf

    def _on_timer(self) -> None:
        from nav_msgs.msg import OccupancyGrid as RosOccupancyGrid
        from std_msgs.msg import Header

        with self._lock:
            pose = self._pose_xyyaw
        if pose is None:
            return
        n_occ, n_inf, occ, inf = self._stats()
        msg = RosOccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        res = self.args.resolution
        rng = self.args.range
        n = int(math.ceil(2.0 * rng / res))
        msg.info.resolution = res
        msg.info.width = n
        msg.info.height = n
        origin_x = pose[0] - rng
        origin_y = pose[1] - rng
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        data = [-1] * (n * n)
        for row in range(n):
            wy = origin_y + (row + 0.5) * res
            for col in range(n):
                wx = origin_x + (col + 0.5) * res
                cell = self.grid.world_to_cell(wx, wy)
                idx = row * n + col
                if cell in occ:
                    data[idx] = 100
                elif cell in inf:
                    data[idx] = 50
                else:
                    val = self.grid.cell_at(*cell)
                    if val > 0:
                        data[idx] = 0
        msg.data = data
        self._pub.publish(msg)
        now = time.monotonic()
        if now - self._last_print >= 1.0:
            self._last_print = now
            print(
                f"pose:\n"
                f"  x={pose[0]:.3f}\n"
                f"  y={pose[1]:.3f}\n"
                f"  yaw={math.degrees(pose[2]):.1f}\n"
                f"cloud_source={self._cloud_source}\n"
                f"raw_points={self._last_raw}\n"
                f"valid_points={self._last_valid}\n"
                f"z_keep_frac={self._z_keep_frac:.2f}\n"
                f"z_range=[{self._z_min_seen:.2f},{self._z_max_seen:.2f}]\n"
                f"occupied_cells={n_occ}\n"
                f"inflated_cells={n_inf}\n"
                f"grid_resolution={res:.2f}\n"
                f"grid_size={n}x{n}\n",
                flush=True,
            )


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="nav_demo occupancy grid (no chassis)")
    p.add_argument("--resolution", type=float, default=0.10)
    p.add_argument("--range", type=float, default=6.0)
    p.add_argument("--min-range", type=float, default=0.45, help="ignore hits inside hull (m)")
    p.add_argument("--inflation", type=float, default=0.35)
    p.add_argument("--ttl", type=float, default=5.0)
    p.add_argument("--z-min", type=float, default=0.15, help="base_link z min (m)")
    p.add_argument("--z-max", type=float, default=1.20, help="base_link z max (m)")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    import rclpy

    rclpy.init()
    app = CloudToGrid(args)
    try:
        rclpy.spin(app.node)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass
    finally:
        try:
            app.node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
