#!/usr/bin/python3
"""Plan A* on live obstacle_grid (2.0 m) and render PNG. No CAN, no motion."""
from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("ROS_HOME", "/tmp/ros_ppt_astar")
os.makedirs(os.environ["ROS_HOME"] + "/log", exist_ok=True)

ROOT = Path("/home/fafu_robot/Desktop/chassis_lidar_drivers")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "nav_demo"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch

from official_mods import load_global_planner
from ros_grid import OCCUPIED, FootprintClearGrid, RosGridAdapter

OUT = ROOT / "ppt_materials" / "generated"
OUT.mkdir(parents=True, exist_ok=True)


def wait_pose_grid(timeout=10.0):
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    qos_tl = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )
    rclpy.init()
    node = Node("ppt_astar_capture")
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
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return box["pose"], box["grid"]


def publish_path(wps):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Header

    if not rclpy.ok():
        rclpy.init()
    node = Node("ppt_astar_path_pub")
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
    for _ in range(6):
        pub.publish(path)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.04)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def path_length(wps) -> float:
    if not wps or len(wps) < 2:
        return 0.0
    s = 0.0
    for i in range(1, len(wps)):
        s += math.hypot(wps[i].x - wps[i - 1].x, wps[i].y - wps[i - 1].y)
    return s


def detour_vs_straight(wps, sx, sy, gx, gy) -> bool:
    if not wps or len(wps) < 2:
        return False
    straight = math.hypot(gx - sx, gy - sy)
    # max lateral offset from the start-goal line
    vx, vy = gx - sx, gy - sy
    nrm = math.hypot(vx, vy) + 1e-9
    max_off = 0.0
    for wp in wps:
        ox, oy = wp.x - sx, wp.y - sy
        # cross / |v|
        off = abs(vx * oy - vy * ox) / nrm
        max_off = max(max_off, off)
    return max_off > 0.25 or path_length(wps) > straight * 1.12


def render(grid_msg, pose, gx, gy, wps, found, path):
    w, h = grid_msg.info.width, grid_msg.info.height
    res = grid_msg.info.resolution
    ox = grid_msg.info.origin.position.x
    oy = grid_msg.info.origin.position.y
    data = np.array(grid_msg.data, dtype=np.int16).reshape((h, w))
    img = np.zeros((h, w, 3), dtype=np.float64)
    img[data < 0] = (0.02, 0.02, 0.02)
    img[data == 0] = (0.92, 0.55, 0.12)
    img[(data > 0) & (data < 100)] = (0.55, 0.22, 0.06)
    img[data >= 100] = (0.82, 0.10, 0.08)
    fig = plt.figure(figsize=(10.5, 9.0), dpi=180, facecolor="black")
    ax = fig.add_subplot(111, facecolor="black")
    ext = [ox, ox + w * res, oy, oy + h * res]
    ax.imshow(img, origin="lower", extent=ext, interpolation="nearest")
    sx, sy, yaw = pose
    if wps:
        xs = [wp.x for wp in wps]
        ys = [wp.y for wp in wps]
        ax.plot(xs, ys, color="#ffe082", lw=2.8, ls="--", zorder=5, label="A* path")
        ax.scatter(xs, ys, c="#fff59d", s=18, zorder=6, edgecolors="#ff6f00")
    ax.plot(sx, sy, marker=(3, 0, math.degrees(yaw) - 90), markersize=18, color="#42a5f5", zorder=7)
    ax.scatter([gx], [gy], marker="*", s=220, c="#ef5350", zorder=7, label="goal")
    ax.annotate("start", (sx, sy), textcoords="offset points", xytext=(8, 8), color="#90caf9", fontsize=9)
    ax.annotate("goal 2.0 m", (gx, gy), textcoords="offset points", xytext=(8, -12), color="#ef9a9a", fontsize=9)
    ax.set_aspect("equal")
    ax.set_xlabel("map X (m)", color="white")
    ax.set_ylabel("map Y (m)", color="white")
    ax.tick_params(colors="white")
    for sp in ax.spines.values():
        sp.set_color("#555555")
    title = "Real A* on obstacle_grid   (plan only, not executed)"
    if wps:
        title += f"   found={found}  {path_length(wps):.2f} m"
    else:
        title = "Real obstacle_grid   occupied / inflated / free"
    ax.set_title(title, color="white")
    ax.legend(
        handles=[
            Patch(facecolor=(0.82, 0.10, 0.08), label="occupied"),
            Patch(facecolor=(0.55, 0.22, 0.06), label="inflated"),
            Patch(facecolor=(0.92, 0.55, 0.12), label="free"),
        ],
        loc="upper right",
        fontsize=8,
        facecolor="#111111",
        labelcolor="white",
        edgecolor="#333333",
    )
    ys, xs = np.where((data >= 0))
    if xs.size:
        m = 0.8
        ax.set_xlim(ox + xs.min() * res - m, ox + (xs.max() + 1) * res + m)
        ax.set_ylim(oy + ys.min() * res - m, oy + (ys.max() + 1) * res + m)
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight", facecolor="black")
    plt.close(fig)


def main() -> int:
    print("PPT A* capture — plan only, no CAN, no motion", flush=True)
    pose, grid = wait_pose_grid(12.0)
    if pose is None:
        print("FAIL: no pose")
        return 2
    if grid is None:
        print("FAIL: no obstacle_grid")
        return 2
    sx, sy, yaw = pose
    gx = sx + 2.0 * math.cos(yaw)
    gy = sy + 2.0 * math.sin(yaw)
    raw = RosGridAdapter(grid)
    adapter = FootprintClearGrid(raw, sx, sy, radius_m=0.40)
    gp_mod = load_global_planner()
    planner = gp_mod.GlobalPlanner(adapter, inflation_m=0.20)
    start_cell = adapter.world_to_cell(sx, sy)
    goal_cell = adapter.world_to_cell(gx, gy)
    print(f"START cell={start_cell} val={adapter.cell_at(*start_cell)}")
    print(f"GOAL  cell={goal_cell} val={adapter.cell_at(*goal_cell)}")
    wps = planner.plan(sx, sy, gx, gy)
    goal_note = "heading_2.0m"
    if wps is None:
        # heading goal in occupied/inflation/unknown: pick nearest free cell ~2 m
        best = None
        for (cx, cy), val in adapter.iter_cells():
            if val >= OCCUPIED:
                continue
            wx = (cx + 0.5) * adapter.resolution_m
            wy = (cy + 0.5) * adapter.resolution_m
            d = math.hypot(wx - sx, wy - sy)
            if d < 1.5 or d > 4.0:
                continue
            heading = abs(
                math.atan2(wy - sy, wx - sx) - yaw
            )
            heading = min(heading, 2 * math.pi - heading)
            score = abs(d - 2.0) + 0.6 * heading
            if best is None or score < best[0]:
                best = (score, wx, wy, d)
        if best is not None:
            gx, gy = best[1], best[2]
            goal_note = f"snapped_free d={best[3]:.2f}m"
            print(f"HEADING_GOAL blocked; snap to free ({gx:.3f},{gy:.3f}) {goal_note}")
            wps = planner.plan(sx, sy, gx, gy)
    found = wps is not None
    n = 0 if not found else len(wps)
    plen = path_length(wps) if found else 0.0
    detour = detour_vs_straight(wps, sx, sy, gx, gy) if found else False
    print(f"POSE x={sx:.3f} y={sy:.3f} yaw_deg={math.degrees(yaw):.1f}")
    print(f"GOAL x={gx:.3f} y={gy:.3f} forward=2.0")
    print(f"PLAN found={found} waypoints={n} path_length={plen:.3f} detour={int(detour)}")
    if found:
        for i, wp in enumerate(wps):
            print(f"  wp[{i}] x={wp.x:.3f} y={wp.y:.3f}")
        try:
            publish_path(wps)
            print("published /nav_demo/plan (latched, not executed)")
        except Exception as exc:
            print(f"WARN publish path skipped: {exc}")
    out = OUT / "astar_path.png"
    render(grid, pose, gx, gy, wps or [], found, out)
    print(f"WROTE {out}")
    grid_out = OUT / "obstacle_grid.png"
    render(grid, pose, gx, gy, [], False, grid_out)
    print(f"WROTE {grid_out}")
    (OUT / "astar_meta.txt").write_text(
        f"found={found}\nwaypoints={n}\npath_length_m={plen:.4f}\ndetour={detour}\n"
        f"start={sx:.4f},{sy:.4f}\ngoal={gx:.4f},{gy:.4f}\nforward_m=2.0\n"
        f"goal_note={goal_note}\nexecuted=False\ncan_sent=False\n"
    )
    return 0 if found else 3


if __name__ == "__main__":
    sys.exit(main())
