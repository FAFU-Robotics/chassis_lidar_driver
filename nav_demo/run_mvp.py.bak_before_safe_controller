#!/usr/bin/env python3
"""Stage 4: Grid → A* → Navigator.goto with MOLA pose + ObstacleGuard. Dry-run only."""
from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(HERE))

from mola_pose import (  # noqa: E402
    GridWatch,
    MolaPoseWatch,
    NodeSpinThread,
    format_pose_rx,
    qos_mola_pose,
)
from official_mods import load_global_planner, load_navigator, load_obstacle  # noqa: E402
from ros_grid import OCCUPIED, FootprintClearGrid, RosGridAdapter  # noqa: E402


class SectorLidar:
    """Duck-typed lidar for ObstacleGuard. No UDP 6699. Sectors from live grid cells."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sectors: list[tuple[float, float]] = []
        self._stamp = 0.0

    def update_from_grid(self, adapter: RosGridAdapter, x: float, y: float, yaw: float) -> None:
        pts: list[tuple[float, float]] = []
        cy, sy = math.cos(yaw), math.sin(yaw)
        for (cx, cy_cell), val in adapter.iter_cells():
            if val < OCCUPIED:
                continue
            wx = (cx + 0.5) * adapter.resolution_m
            wy = (cy_cell + 0.5) * adapter.resolution_m
            dx, dy = wx - x, wy - y
            fwd = dx * cy + dy * sy
            left = -dx * sy + dy * cy
            right = -left
            dist = math.hypot(right, fwd)
            if dist < 0.12 or dist > 6.0:
                continue
            az = math.degrees(math.atan2(right, fwd))
            pts.append((az, dist))
        with self._lock:
            self._sectors = pts
            self._stamp = time.monotonic()

    @property
    def is_receiving(self) -> bool:
        return time.monotonic() - self._stamp < 2.0

    @property
    def latest_frame(self):
        return None

    def sector_points(self) -> list[tuple[float, float]]:
        with self._lock:
            return list(self._sectors)

    def nearest_in_range(self, center_deg: float, width_deg: float, quantile: float = 0.25):
        half = width_deg * 0.5
        hits = []
        with self._lock:
            secs = list(self._sectors)
        for az, dist in secs:
            d = (az - center_deg + 180.0) % 360.0 - 180.0
            if abs(d) <= half:
                hits.append(dist)
        if not hits:
            return None
        hits.sort()
        q = min(max(quantile, 0.0), 1.0)
        idx = min(len(hits) - 1, max(0, int(round((len(hits) - 1) * q))))
        return hits[idx]


class DryRunController:
    def set_velocity(self, linear_m_s: float, angular_rad_s: float) -> None:
        print(f"DRY_RUN v={linear_m_s:.3f} w={angular_rad_s:.3f}", flush=True)

    def stop_motion(self) -> None:
        print("DRY_RUN v=0.000 w=0.000 abort/stop", flush=True)


def yaw_from_q(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main() -> int:
    ap = argparse.ArgumentParser(
        description=(
            "nav_demo Navigator (dry-run default; --live one-pulse; "
            "--live-go 1m closed loop; --live-avoid 2m real-grid avoid)"
        )
    )
    ap.add_argument("--forward", type=float, default=1.8)
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--speed", type=float, default=0.12)
    ap.add_argument("--inflation", type=float, default=0.20)
    ap.add_argument(
        "--live",
        action="store_true",
        help=(
            "one chassis pulse via TeleopTcpClient → :9100 "
            "(v=0.06 w=0 duration=1.00s, then observe pose >=2s, then halt)"
        ),
    )
    ap.add_argument(
        "--live-go",
        action="store_true",
        help=(
            "closed-loop 1.0 m straight at v=0.06 w=0 via repeated 0.30s pulses "
            "until ARRIVE_M=0.18; independent of --live one-pulse"
        ),
    )
    ap.add_argument(
        "--live-avoid",
        action="store_true",
        help=(
            "closed-loop ~2.0 m on real obstacle_grid; Guard BLOCK then A* replan; "
            "independent of --live-go"
        ),
    )
    args = ap.parse_args()
    n_live = int(args.live) + int(args.live_go) + int(args.live_avoid)
    if n_live > 1:
        print("FAIL: use only one of --live / --live-go / --live-avoid")
        return 2
    if args.live:
        print(
            "run_mvp --live: ONE pulse v=0.06 w=0 duration=1.00s; "
            "keep ROS spinner/pose >=2s after pulse; then stick(0,0)+estop; "
            "POSE_STALE_S=0.50; no repeat",
            flush=True,
        )
        argv_saved = sys.argv
        # Strip --live; one-pulse is implemented in run_first_live (nav_demo only).
        sys.argv = [argv_saved[0], "--one-pulse"]
        try:
            from run_first_live import main as live_main

            return live_main()
        finally:
            sys.argv = argv_saved
    if args.live_go:
        print(
            "run_mvp --live-go: closed-loop v=0.06 w=0 to 1.0m ARRIVE_M=0.18; "
            "pulse=0.30s period=0.18s; POSE_STALE_S=0.50; real MOLA pose only",
            flush=True,
        )
        from run_live_go import run_live_go

        return run_live_go()
    if args.live_avoid:
        print(
            "run_mvp --live-avoid: ~2.0 m real-grid avoid; pulse=0.30s; "
            "Guard BLOCK → A* replan; POSE_STALE_S=0.50; does not change --live-go",
            flush=True,
        )
        from run_live_avoid import run_live_avoid

        return run_live_avoid()
    print("STAGE4 NAVIGATOR — DRY_RUN only. no stick, no CAN, no chassis", flush=True)

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from rclpy.node import Node
    from std_msgs.msg import Header

    rclpy.init()
    node = Node("nav_demo_run_mvp")
    pose_watch = MolaPoseWatch(node)
    grid_watch = GridWatch(node)
    path_pub = node.create_publisher(Path, "/nav_demo/plan", qos_mola_pose())
    spinner = NodeSpinThread(node)
    spinner.start()

    def ros_cleanup() -> None:
        spinner.stop()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    t0 = time.monotonic()
    while time.monotonic() - t0 < 8.0:
        snap_w = pose_watch.snapshot()
        gmsg_w, _n_gw, g_age_w = grid_watch.snapshot()
        if (
            snap_w is not None
            and gmsg_w is not None
            and snap_w.n >= 3
            and snap_w.age_s < 0.30
            and g_age_w < 0.80
        ):
            break
        time.sleep(0.05)
    pose0 = pose_watch.snapshot()
    grid0, n_grid0, _g_age = grid_watch.snapshot()
    if pose0 is None or grid0 is None:
        print("FAIL: missing pose or grid")
        ros_cleanup()
        return 2

    state = {
        "pose": (pose0.x, pose0.y, pose0.yaw),
        "grid": grid0,
        "n_pose": pose0.n,
        "n_grid": n_grid0,
    }

    sx, sy, yaw = state["pose"]
    gx = sx + args.forward * math.cos(yaw)
    gy = sy + args.forward * math.sin(yaw)
    raw = RosGridAdapter(state["grid"])
    adapter = FootprintClearGrid(raw, sx, sy, radius_m=0.40)
    print(f"POSE x={sx:.3f} y={sy:.3f} yaw_deg={math.degrees(yaw):.1f}", flush=True)
    print(f"GRID occupied={adapter.occupied_count()} free={adapter.free_count()}", flush=True)

    gp_mod = load_global_planner()
    nav_mod = load_navigator()
    obs_mod = load_obstacle()
    planner = gp_mod.GlobalPlanner(adapter, inflation_m=args.inflation)
    wps = planner.plan(sx, sy, gx, gy)
    found = wps is not None
    print(f"PLAN found={found} waypoints={0 if not wps else len(wps)}", flush=True)
    if not found:
        print("FAIL: A* none — Navigator not started")
        ros_cleanup()
        return 3
    for i, wp in enumerate(wps):
        print(f"WAYPOINT i={i} x={wp.x:.3f} y={wp.y:.3f}", flush=True)

    path = Path()
    path.header = Header()
    path.header.stamp = node.get_clock().now().to_msg()
    path.header.frame_id = "map"
    for wp in wps:
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = wp.x
        ps.pose.position.y = wp.y
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    path_pub.publish(path)

    lidar = SectorLidar()
    lidar.update_from_grid(adapter, sx, sy, yaw)
    guard = obs_mod.ObstacleGuard(lidar, require_sensor=False)
    ctrl = DryRunController()
    cfg = nav_mod.NavigateConfig(
        max_linear_m_s=max(0.12, args.speed),
        max_angular_rad_s=0.40,
        goal_tolerance_m=0.12,
        stall_timeout_s=8.0,
        replan_after_s=3.0,
        update_interval_s=0.05,
    )

    last_guard = {"v": 0.0, "w": 0.0, "blocked": False}
    replan_n = {"n": 0}

    def drive(v, w):
        gv, gw, blocked = guard.guard_velocity(v, w)
        last_guard["v"], last_guard["w"], last_guard["blocked"] = gv, gw, blocked
        print(
            f"GUARD blocked={int(blocked)} in_v={v:.3f} in_w={w:.3f} out_v={gv:.3f} out_w={gw:.3f}",
            flush=True,
        )
        print(f"DRY_RUN v={gv:.3f} w={gw:.3f}", flush=True)
        # never TeleopTcpClient.stick

    def replanner(goal_x: float, goal_y: float):
        replan_n["n"] += 1
        pose_now = state["pose"]
        grid_now = state["grid"]
        if pose_now is None or grid_now is None:
            print("REPLAN fail missing pose/grid", flush=True)
            return None
        ad = FootprintClearGrid(RosGridAdapter(grid_now), pose_now[0], pose_now[1], radius_m=0.40)
        pl = gp_mod.GlobalPlanner(ad, inflation_m=args.inflation)
        path2 = pl.plan(pose_now[0], pose_now[1], goal_x, goal_y)
        n = 0 if path2 is None else len(path2)
        print(f"REPLAN n={replan_n['n']} found={path2 is not None} waypoints={n}", flush=True)
        if path2 is None:
            return None
        mids = [(wp.x, wp.y) for wp in path2[1:-1]]
        return mids

    nav = nav_mod.Navigator(ctrl, guard=guard, config=cfg, drive=drive)
    nav.apply_external_pose(sx, sy, math.degrees(yaw))
    print("POSE source=MOLA_map apply_external_pose (not wheel odom)", flush=True)

    mids = [(wp.x, wp.y) for wp in wps[1:-1]]
    probe = replanner(gx, gy)
    print(f"REPLAN wired probe_ok={probe is not None}", flush=True)

    ok = nav.goto(
        gx,
        gy,
        speed=args.speed,
        waypoints=mids,
        replanner=replanner,
        on_abort=lambda reason: print(f"NAV abort={reason}", flush=True),
        on_arrived=lambda: print("NAV arrived (pose still; dry-run may not arrive)", flush=True),
    )
    print(f"NAVIGATOR goto_started={ok} goal=({gx:.3f},{gy:.3f})", flush=True)

    t_end = time.monotonic() + args.seconds
    last_rx = 0.0
    last_print = 0.0
    main_tick = time.monotonic()
    while time.monotonic() < t_end:
        snap = pose_watch.snapshot()
        gmsg, n_g, _g_age = grid_watch.snapshot()
        if snap is not None:
            state["pose"] = (snap.x, snap.y, snap.yaw)
            state["n_pose"] = snap.n
        if gmsg is not None:
            state["grid"] = gmsg
            state["n_grid"] = n_g
        pose = state["pose"]
        grid = state["grid"]
        if pose is not None:
            nav.apply_external_pose(pose[0], pose[1], math.degrees(pose[2]))
            if grid is not None:
                ad = RosGridAdapter(grid)
                lidar.update_from_grid(ad, pose[0], pose[1], pose[2])
        now = time.monotonic()
        if now - last_rx >= 0.25:
            last_rx = now
            print(format_pose_rx(snap, spinner, main_tick, stale_s=0.50), flush=True)
        if now - last_print >= 1.0 and pose is not None:
            last_print = now
            print(
                f"POSE x={pose[0]:.3f} y={pose[1]:.3f} yaw_deg={math.degrees(pose[2]):.1f} "
                f"n_pose={state['n_pose']} age_s={-1 if snap is None else snap.age_s:.3f} "
                f"GRID n={state['n_grid']} navigating={nav.is_navigating} "
                f"GUARD blocked={int(last_guard['blocked'])} REPLAN n={replan_n['n']}",
                flush=True,
            )
            path_pub.publish(path)
        main_tick = now
        time.sleep(0.05)

    print("STOP abort Navigator (dry-run)", flush=True)
    nav.stop()
    print(
        f"PASS_INTERFACE POSE GRID PLAN WAYPOINT GUARD REPLAN DRY_RUN "
        f"replan_calls={replan_n['n']} last_dry_v={last_guard['v']:.3f} last_dry_w={last_guard['w']:.3f}",
        flush=True,
    )
    ros_cleanup()
    return 0


if __name__ == "__main__":
    sys.exit(main())
