#!/usr/bin/env python3
"""Stage 3: real /nav_demo/obstacle_grid + MOLA pose + test goal → official A*. No chassis."""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(HERE))

from official_mods import load_global_planner  # noqa: E402
from ros_grid import OCCUPIED, FootprintClearGrid, RosGridAdapter  # noqa: E402


def _wait_pose_grid(timeout: float):
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = Node("nav_demo_plan_once")
    qos_tl = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )
    box = {"pose": None, "grid": None}

    def on_pose(msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        box["pose"] = (float(p.x), float(p.y), float(yaw))

    def on_grid(msg):
        box["grid"] = msg

    node.create_subscription(Odometry, "/lidar_odometry/pose", on_pose, qos_tl)
    node.create_subscription(OccupancyGrid, "/nav_demo/obstacle_grid", on_grid, qos_tl)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if box["pose"] is not None and box["grid"] is not None:
            break
    pose, grid = box["pose"], box["grid"]
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return pose, grid


def _path_cells(planner, wps) -> list[tuple[int, int]]:
    cells = []
    for wp in wps:
        cells.append(planner._grid.world_to_cell(wp.x, wp.y))
    return cells


def _bresenham(a, b):
    x0, y0 = a
    x1, y1 = b
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    out = []
    while True:
        out.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="nav_demo A* dry-run (no stick)")
    ap.add_argument("--forward", type=float, default=1.8, help="goal distance along heading (m)")
    ap.add_argument("--gx", type=float, default=None, help="goal X in map (m); with --gy overrides --forward")
    ap.add_argument("--gy", type=float, default=None, help="goal Y in map (m); with --gx overrides --forward")
    ap.add_argument("--inflation", type=float, default=0.20)
    ap.add_argument("--timeout", type=float, default=8.0)
    args = ap.parse_args()
    if (args.gx is None) ^ (args.gy is None):
        print("FAIL: --gx and --gy must be given together")
        return 2

    print("STAGE3 PLAN — no stick, no CAN, no chassis", flush=True)
    pose, grid_msg = _wait_pose_grid(args.timeout)
    if pose is None:
        print("FAIL: no /lidar_odometry/pose")
        return 2
    if grid_msg is None:
        print("FAIL: no /nav_demo/obstacle_grid")
        return 2

    sx, sy, yaw = pose
    if args.gx is not None:
        gx, gy = float(args.gx), float(args.gy)
        print(f"GOAL_MODE map_xy ({gx:.3f},{gy:.3f})", flush=True)
    else:
        gx = sx + args.forward * math.cos(yaw)
        gy = sy + args.forward * math.sin(yaw)
        print(f"GOAL_MODE heading_{args.forward:.1f}m", flush=True)
    raw = RosGridAdapter(grid_msg)
    adapter = FootprintClearGrid(raw, sx, sy, radius_m=0.40)
    gp_mod = load_global_planner()
    planner = gp_mod.GlobalPlanner(adapter, inflation_m=args.inflation)

    start_cell = adapter.world_to_cell(sx, sy)
    goal_cell = adapter.world_to_cell(gx, gy)
    nearest_occ = None
    for (cx, cy), val in adapter.iter_cells():
        if val < OCCUPIED:
            continue
        wx = (cx + 0.5) * adapter.resolution_m
        wy = (cy + 0.5) * adapter.resolution_m
        d = math.hypot(wx - sx, wy - sy)
        if nearest_occ is None or d < nearest_occ:
            nearest_occ = d
    print(f"POSE x={sx:.3f} y={sy:.3f} yaw_deg={math.degrees(yaw):.1f}")
    print(f"GRID occupied={adapter.occupied_count()} free={adapter.free_count()} res={adapter.resolution_m:.2f}")
    print(f"START world=({sx:.3f},{sy:.3f}) cell={start_cell} val={adapter.cell_at(*start_cell)}")
    print(f"GOAL  world=({gx:.3f},{gy:.3f}) cell={goal_cell} val={adapter.cell_at(*goal_cell)}")
    print(f"NEAREST_OCC_TO_START_m={nearest_occ}")

    wps = planner.plan(sx, sy, gx, gy)
    found = wps is not None
    print(f"PLAN found={found}")
    if not found:
        print("WAYPOINTS n=0")
        print("FAIL: A* returned None (start/goal in obstacle/inflation, or no path)")
        print(f"  start_traversable_check cell_val={adapter.cell_at(*start_cell)}")
        print(f"  goal_traversable_check cell_val={adapter.cell_at(*goal_cell)}")
        print(f"  inflated_cells={planner.inflated_cell_count}")
        return 3

    print(f"WAYPOINTS n={len(wps)}")
    for i, wp in enumerate(wps):
        print(f"  wp[{i}] x={wp.x:.3f} y={wp.y:.3f}")

    crossed_occ = 0
    crossed_inf = 0
    inflated = planner._inflated or frozenset()
    cells = _path_cells(planner, wps)
    for i in range(len(cells) - 1):
        for c in _bresenham(cells[i], cells[i + 1]):
            if adapter.cell_at(*c) >= OCCUPIED:
                crossed_occ += 1
            if c in inflated:
                crossed_inf += 1
    print(f"PATH_CROSS occupied_hits={crossed_occ} inflated_hits={crossed_inf}")
    print(f"PLAN inflated_cells={planner.inflated_cell_count}")
    if crossed_occ:
        print("FAIL: path crosses occupied cells")
        return 4
    if crossed_inf:
        print("FAIL: path crosses inflated cells")
        return 4
    print("PASS: A* path found and does not cross occupied/inflated")
    _publish_path(wps)
    return 0


def _publish_path(wps) -> None:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Header

    if not rclpy.ok():
        rclpy.init()
    node = Node("nav_demo_plan_once_pub")
    qos_tl = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )
    pub = node.create_publisher(Path, "/nav_demo/plan", qos_tl)
    path = Path()
    path.header = Header()
    path.header.stamp = node.get_clock().now().to_msg()
    path.header.frame_id = "map"
    for wp in wps:
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = float(wp.x)
        ps.pose.position.y = float(wp.y)
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    for _ in range(8):
        pub.publish(path)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)
    print(f"published /nav_demo/plan poses={len(path.poses)}", flush=True)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
