#!/usr/bin/env python3
"""nav_demo live-avoid. Independent of --live-go (do not change live-go).

Real /rslidar_points + /lidar_odometry/pose + /nav_demo/obstacle_grid → A*.
Follows the full waypoint list (not start→goal 2-point). v<=0.06, |w|<=0.25.
Guard BLOCK → stop → replan on latest real pose+grid. Pose is MOLA only.
"""
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

from official_mods import (  # noqa: E402
    ensure_package,
    load_global_planner,
    load_navigator,
    load_obstacle,
)
from mola_pose import (  # noqa: E402
    GridWatch,
    MolaPoseWatch,
    NodeSpinThread,
    classify_pose_rx,
    format_pose_rx,
    qos_mola_pose,
)
from ros_grid import FootprintClearGrid, RosGridAdapter  # noqa: E402
from run_first_live import (  # noqa: E402
    ARRIVE_M,
    GRID_STALE_S,
    LATERAL_ABORT_M,
    POSE_HOLD_ABORT_S,
    POSE_STALE_S,
    V_CMD,
    YAW_ABORT_DEG,
    Halt,
    can_control_mode,
    halt_chassis,
    query_vw,
    track_progress,
    wrap_deg,
)
from run_live_go import ControlModeWatch, LIVE_GO_PERIOD_S, LIVE_GO_PULSE_S  # noqa: E402
from run_mvp import SectorLidar  # noqa: E402

AVOID_GOAL_FWD_M = 2.0  # behind the ~1.1 m box so A* can return a detour
AVOID_TIMEOUT_S = 50.0
AVOID_W_MAX = 0.25
AVOID_WP_REACH_M = 0.28
AVOID_TURN_RAD = 0.40
AVOID_INFLATION_M = 0.20


def wrap_rad(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def path_len(wps) -> float:
    if not wps or len(wps) < 2:
        return 0.0
    return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(wps, wps[1:]))


def plan_on_grid(gp_mod, grid_msg, px, py, gx, gy):
    ad = FootprintClearGrid(RosGridAdapter(grid_msg), px, py, radius_m=0.40)
    planner = gp_mod.GlobalPlanner(ad, inflation_m=AVOID_INFLATION_M)
    return planner.plan(px, py, gx, gy), ad


def chase_vw(px, py, yaw, tx, ty) -> tuple[float, float]:
    """Legacy combined v/w chase. Not used by live-avoid (ALIGN/DRIVE instead)."""
    heading = math.atan2(ty - py, tx - px)
    err = wrap_rad(heading - yaw)
    w = max(-AVOID_W_MAX, min(AVOID_W_MAX, 1.2 * err))
    if abs(err) >= AVOID_TURN_RAD:
        v = 0.0
    else:
        v = V_CMD * max(0.0, 1.0 - abs(err) / AVOID_TURN_RAD)
        v = min(V_CMD, max(0.0, v))
    return v, w


ALIGN_ERR_RAD = math.radians(5.0)
REENTER_ALIGN_RAD = math.radians(8.0)
DRIVE_V_MAX = 0.05
ALIGN_W_MAX = 0.20
DRIVE_W_MAX = 0.10


def replan_none_must_stop(path2) -> bool:
    """A* none → SAFE_STOP. Never keep old waypoints. Returns True if stopped."""
    if path2 is not None:
        return False
    print("REPLAN_FAILED A* none", flush=True)
    print("REPLAN_FAILED -> SAFE_STOP", flush=True)
    print("NO_DRIVE after replan fail (old waypoints discarded)", flush=True)
    return True


def heading_err(px, py, yaw, tx, ty) -> float:
    return wrap_rad(math.atan2(ty - py, tx - px) - yaw)


def align_drive_vw(px, py, yaw, tx, ty, phase: dict) -> tuple[float, float, float, str]:
    """Two-stage: ALIGN (v=0) then DRIVE (v<=0.05). Re-enter ALIGN if |err|>8°."""
    err = heading_err(px, py, yaw, tx, ty)
    err_deg = math.degrees(err)
    name = phase.get("name") or "ALIGN"
    if abs(err) > REENTER_ALIGN_RAD:
        name = "ALIGN"
    elif abs(err) <= ALIGN_ERR_RAD:
        name = "DRIVE"
    phase["name"] = name
    if name == "ALIGN":
        w = max(-ALIGN_W_MAX, min(ALIGN_W_MAX, 1.2 * err))
        return 0.0, w, err_deg, "ALIGN"
    w = max(-DRIVE_W_MAX, min(DRIVE_W_MAX, 0.8 * err))
    return DRIVE_V_MAX, w, err_deg, "DRIVE"


def paint_aabb(msg, xmin, xmax, ymin, ymax, value=100) -> int:
    res = float(msg.info.resolution)
    ox = float(msg.info.origin.position.x)
    oy = float(msg.info.origin.position.y)
    w = int(msg.info.width)
    h = int(msg.info.height)
    data = list(msg.data)
    n = 0
    col0 = max(0, int(math.floor((xmin - ox) / res)))
    col1 = min(w - 1, int(math.floor((xmax - ox) / res)))
    row0 = max(0, int(math.floor((ymin - oy) / res)))
    row1 = min(h - 1, int(math.floor((ymax - oy) / res)))
    for row in range(row0, row1 + 1):
        for col in range(col0, col1 + 1):
            data[row * w + col] = value
            n += 1
    msg.data = data
    return n


def path_hits_aabb(wps, aabb, margin=0.05) -> bool:
    if not wps:
        return True
    xmin, xmax, ymin, ymax = aabb
    xmin -= margin
    xmax += margin
    ymin -= margin
    ymax += margin
    for a, b in zip(wps, wps[1:]):
        steps = max(2, int(math.hypot(b.x - a.x, b.y - a.y) / 0.05))
        for i in range(steps + 1):
            t = i / steps
            x = a.x + t * (b.x - a.x)
            y = a.y + t * (b.y - a.y)
            if xmin <= x <= xmax and ymin <= y <= ymax:
                return True
    return False


def dry_run_avoid() -> int:
    """Prove A* / Guard BLOCK / replan / v,w. Never TeleopTcpClient / CAN."""
    print("LIVE_AVOID dry-run: no stick, no move, no CAN, no --live-avoid", flush=True)
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("nav_demo_live_avoid_dry")
    pose_watch = MolaPoseWatch(node)
    grid_watch = GridWatch(node)
    spinner = NodeSpinThread(node)
    spinner.start()
    t0 = time.monotonic()
    snap = grid = None
    while time.monotonic() - t0 < 6.0:
        s = pose_watch.snapshot()
        g, _n, age = grid_watch.snapshot()
        if s is not None and g is not None and s.n >= 3 and s.age_s < 0.30 and age < 0.80:
            snap, grid = s, g
            break
        time.sleep(0.05)
    if snap is None or grid is None:
        print("FAIL DRY missing live pose/grid")
        spinner.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 2

    sx, sy, yaw = snap.x, snap.y, snap.yaw
    gx = sx + AVOID_GOAL_FWD_M * math.cos(yaw)
    gy = sy + AVOID_GOAL_FWD_M * math.sin(yaw)
    print("LIVE_AVOID_START (dry-run)")
    print(f"start pose=({sx:.3f},{sy:.3f},yaw_deg={math.degrees(yaw):.2f})")
    print(f"goal=({gx:.3f},{gy:.3f}) forward={AVOID_GOAL_FWD_M:.2f}m")
    print(format_pose_rx(snap, spinner, time.monotonic(), stale_s=POSE_STALE_S))

    gp_mod = load_global_planner()
    nav_mod = load_navigator()
    obs_mod = load_obstacle()
    wps, ad = plan_on_grid(gp_mod, grid, sx, sy, gx, gy)
    found = wps is not None
    n = 0 if wps is None else len(wps)
    print(
        f"PLAN found={found} waypoints={n} length={path_len(wps):.3f}m "
        f"occupied={ad.occupied_count()} (REAL grid, no TEST)",
        flush=True,
    )
    if not found:
        print("FAIL DRY A* none on real grid")
        spinner.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 3
    print("WAYPOINT_ORDER (A* → follow seq ALIGN then DRIVE)")
    for i, wp in enumerate(wps):
        print(f"  WAYPOINT seq={i} x={wp.x:.3f} y={wp.y:.3f}")
    mids = [(wp.x, wp.y) for wp in wps[1:-1]]
    print(f"NAV_GOTO goal=({gx:.3f},{gy:.3f}) n_wp={n} n_mids={len(mids)} mids={mids}")
    ctrl = type("C", (), {"set_velocity": lambda self, v, w: None, "stop_motion": lambda self: None})()
    cfg = nav_mod.NavigateConfig(
        max_linear_m_s=V_CMD,
        max_angular_rad_s=AVOID_W_MAX,
        goal_tolerance_m=ARRIVE_M,
        stall_timeout_s=AVOID_TIMEOUT_S,
        update_interval_s=0.05,
    )
    nav = nav_mod.Navigator(ctrl, guard=None, config=cfg, drive=lambda v, w: None)
    nav.apply_external_pose(sx, sy, math.degrees(yaw))
    nav.goto(gx, gy, speed=V_CMD, waypoints=mids)
    got = list(getattr(nav, "_waypoints", mids))
    print(f"NAV_INTERNAL_WAYPOINTS n={len(got)} {got}")
    try:
        nav.stop()
    except Exception:
        pass
    if n <= 2:
        print("FAIL DRY expected waypoints>2 (2.0m goal should detour the real box)")
        spinner.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 3

    # A: 4-point real path + a 2-point synthetic → ALIGN/DRIVE (no chase_vw, no CAN)
    ph4 = {"name": "ALIGN"}
    tx, ty = wps[1].x, wps[1].y
    v4, w4, e4, st4 = align_drive_vw(sx, sy, yaw, tx, ty, ph4)
    print(f"{st4} wp=1 err={e4:.2f} v={v4:.3f} w={w4:.3f} (4-point, NOT sent)")
    ph2 = {"name": "ALIGN"}
    gx2 = sx + 1.0 * math.cos(yaw)
    gy2 = sy + 1.0 * math.sin(yaw)
    v2a, w2a, e2a, st2a = align_drive_vw(sx, sy, yaw, gx2, gy2, ph2)
    print(f"{st2a} wp=1 err={e2a:.2f} v={v2a:.3f} w={w2a:.3f} (2-point aligned, NOT sent)")
    yaw_off = yaw + math.radians(20.0)
    ph2b = {"name": "ALIGN"}
    v2b, w2b, e2b, st2b = align_drive_vw(sx, sy, yaw_off, gx2, gy2, ph2b)
    print(f"{st2b} wp=1 err={e2b:.2f} v={v2b:.3f} w={w2b:.3f} (2-point 20deg off, NOT sent)")

    lidar = SectorLidar()
    lidar.update_from_grid(RosGridAdapter(grid), sx, sy, yaw)
    guard = obs_mod.ObstacleGuard(lidar, require_sensor=True)
    gv, gw, blocked = guard.guard_velocity(v4, w4)
    print(
        f"GUARD real_grid blocked={int(blocked)} out_v={gv:.3f} out_w={gw:.3f} nearest="
        f"{lidar.nearest_in_range(0.0, 80.0, 0.10)}",
        flush=True,
    )

    # D: A* fail must SAFE_STOP — no DRIVE, no old waypoints
    replan_safe = replan_none_must_stop(None)

    # E: control_mode != 1
    print("CONTROL_MODE_ABORT actual=3 expected=1")
    print("would stick(0,0)+estop then exit (NOT sent)")

    print("DRY_RUN would TeleopTcpClient.cmd(move) duration=0.30 — NOT sent")
    print("FINALLY would halt_chassis stick(0,0)+estop")
    print(f"POSE_STALE_S={POSE_STALE_S:.2f} GRID_STALE_S={GRID_STALE_S:.2f} pulse={LIVE_GO_PULSE_S:.2f}")
    spinner.stop()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    ok_align = st4 == "ALIGN" and v4 == 0.0 and abs(w4) > 1e-6
    ok_drive = st2a == "DRIVE" and 0.0 < v2a <= DRIVE_V_MAX + 1e-9 and abs(w2a) <= DRIVE_W_MAX + 1e-9
    ok_align2 = st2b == "ALIGN" and v2b == 0.0
    ok = found and n > 2 and ok_align and ok_drive and ok_align2 and replan_safe
    print(
        f"DRY_AVOID_CHECK A*={int(found)} waypoints={n} ALIGN={int(ok_align)} "
        f"DRIVE={int(ok_drive)} SAFE_STOP={int(replan_safe)} NO_CAN=1 NO_CHASE_LIVE=1"
    )
    return 0 if ok else 7


def run_live_avoid() -> int:
    print(
        "LIVE_AVOID envelope: 2.0m ALIGN/DRIVE waypoint follow v<=0.05 "
        f"POSE_STALE_S={POSE_STALE_S:.2f} pulse={LIVE_GO_PULSE_S:.2f}s "
        f"period={LIVE_GO_PERIOD_S:.2f}s Guard require_sensor=True "
        "independent of --live-go",
        flush=True,
    )
    import atexit

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from rclpy.node import Node
    from std_msgs.msg import Header

    rclpy.init()
    node = Node("nav_demo_live_avoid")
    pose_watch = MolaPoseWatch(node)
    grid_watch = GridWatch(node)
    path_pub = node.create_publisher(Path, "/nav_demo/plan", qos_mola_pose())
    spinner = NodeSpinThread(node)
    spinner.start()
    mode_watch = ControlModeWatch()
    mode_watch.start()

    def ros_cleanup() -> None:
        mode_watch.stop()
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
    while time.monotonic() - t0 < 6.0:
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
    grid0, n_grid0, _age_g = grid_watch.snapshot()
    if pose0 is None:
        print("FAIL PREFLIGHT no /lidar_odometry/pose")
        ros_cleanup()
        return 2
    if grid0 is None:
        print("FAIL PREFLIGHT no /nav_demo/obstacle_grid")
        ros_cleanup()
        return 2

    state = {
        "pose": (pose0.x, pose0.y, pose0.yaw),
        "grid": grid0,
        "n_pose": pose0.n,
        "n_grid": n_grid0,
    }
    sx, sy, syaw = state["pose"]
    gx = sx + AVOID_GOAL_FWD_M * math.cos(syaw)
    gy = sy + AVOID_GOAL_FWD_M * math.sin(syaw)

    gp_mod = load_global_planner()
    nav_mod = load_navigator()
    obs_mod = load_obstacle()
    wps, adapter = plan_on_grid(gp_mod, state["grid"], sx, sy, gx, gy)
    if not wps:
        print("FAIL PREFLIGHT A* none on real grid")
        ros_cleanup()
        return 3
    print(f"PLAN found=True waypoints={len(wps)} length={path_len(wps):.3f}m", flush=True)
    for i, wp in enumerate(wps):
        print(f"  WAYPOINT seq={i} x={wp.x:.3f} y={wp.y:.3f}", flush=True)

    raw = RosGridAdapter(state["grid"])
    lidar = SectorLidar()
    lidar.update_from_grid(raw, sx, sy, syaw)
    nearest = lidar.nearest_in_range(0.0, 80.0, 0.10)
    print(f"PREFLIGHT POSE x={sx:.3f} y={sy:.3f} yaw_deg={math.degrees(syaw):.2f}", flush=True)
    print(f"PREFLIGHT GRID occupied={adapter.occupied_count()} nearest_fwd={nearest}", flush=True)
    if nearest is not None and nearest < 0.50:
        print(f"FAIL PREFLIGHT obstacle too close ({nearest:.2f} m); not starting")
        ros_cleanup()
        return 4

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

    guard = obs_mod.ObstacleGuard(lidar, require_sensor=True)
    waypoints: list[tuple[float, float]] = [(wp.x, wp.y) for wp in wps]
    wp_i = {"i": 1 if len(waypoints) > 1 else 0}
    phase = {"name": "ALIGN"}
    last_wi = {"i": -1}
    stats = {
        "n_pulse": 0,
        "replan_n": 0,
        "guard_hit": False,
        "arrived": False,
        "reason": "",
        "guard_reason": "",
        "max_age": 0.0,
        "n_pose_end": 0,
        "hold_n": 0,
        "hold_max_s": 0.0,
        "sent_can": False,
    }
    live = {"on": False}
    cli_box: dict = {"cli": None}
    halt_ev = threading.Event()
    halt_reason = {"r": ""}
    pose_hold = {"on": False, "t0": 0.0}
    main_tick = {"t": time.monotonic()}
    last_rx_log = {"t": 0.0, "state": ""}

    def request_halt(reason: str) -> None:
        if halt_ev.is_set():
            return
        halt_reason["r"] = reason
        halt_ev.set()
        print(f"LIVE_AVOID_ABORT reason={reason}", flush=True)
        if live["on"]:
            halt_chassis(cli_box.get("cli"), reason)

    def pull_obs():
        snap = pose_watch.snapshot()
        gmsg, n_g, g_age = grid_watch.snapshot()
        if snap is not None:
            state["pose"] = (snap.x, snap.y, snap.yaw)
            state["n_pose"] = snap.n
        if gmsg is not None:
            state["grid"] = gmsg
            state["n_grid"] = n_g
        return snap, g_age

    def pose_gate(snap) -> str:
        now = time.monotonic()
        spin_age = spinner.spin_age_s()
        main_age = now - main_tick["t"] if main_tick["t"] > 0 else 0.0
        age = float("inf") if snap is None else snap.age_s
        rx_state = classify_pose_rx(age, spin_age, main_age, stale_s=POSE_STALE_S)
        if now - last_rx_log["t"] >= 0.25 or rx_state != last_rx_log["state"]:
            print(format_pose_rx(snap, spinner, main_tick["t"], stale_s=POSE_STALE_S), flush=True)
            last_rx_log["t"] = now
            last_rx_log["state"] = rx_state
        main_tick["t"] = now
        if rx_state == "NO_SPIN":
            raise Halt("ROS spinner dead — pose callbacks cannot run")
        if rx_state == "STALE_MOLA":
            if not pose_hold["on"]:
                pose_hold["on"] = True
                pose_hold["t0"] = now
                stats["hold_n"] += 1
                print("POSE_HOLD enter (motion inhibited; last real pose kept)", flush=True)
            held = now - pose_hold["t0"]
            stats["hold_max_s"] = max(stats["hold_max_s"], held)
            if held > POSE_HOLD_ABORT_S:
                raise Halt(f"MOLA pose dead held={held:.3f}s")
            return "HOLD"
        if pose_hold["on"]:
            pose_hold["on"] = False
            print("POSE_HOLD exit", flush=True)
        return "OK"

    def current_target(px, py) -> tuple[float, float, int]:
        i = wp_i["i"]
        while i < len(waypoints) - 1:
            tx, ty = waypoints[i]
            if math.hypot(tx - px, ty - py) <= AVOID_WP_REACH_M:
                print(f"WAYPOINT_REACH seq={i} x={tx:.3f} y={ty:.3f}", flush=True)
                i += 1
                wp_i["i"] = i
                continue
            return tx, ty, i
        return gx, gy, max(i, len(waypoints) - 1)

    def segment_yaw_xy():
        i = wp_i["i"]
        if i <= 0 or len(waypoints) < 2:
            return sx, sy, syaw
        ax, ay = waypoints[max(0, i - 1)]
        bx, by = waypoints[min(i, len(waypoints) - 1)]
        return ax, ay, math.atan2(by - ay, bx - ax)

    def replanner(goal_x: float, goal_y: float):
        stats["replan_n"] += 1
        nowp = pose_watch.snapshot()
        gmsg, _n, _a = grid_watch.snapshot()
        if nowp is None or gmsg is None:
            print("REPLAN_FAILED missing pose/grid", flush=True)
            print("REPLAN_FAILED -> SAFE_STOP", flush=True)
            request_halt("REPLAN_FAILED -> SAFE_STOP")
            return None
        print(
            f"REPLAN n={stats['replan_n']} from=({nowp.x:.3f},{nowp.y:.3f}) "
            f"to=({goal_x:.3f},{goal_y:.3f}) REAL grid",
            flush=True,
        )
        path2, _ = plan_on_grid(gp_mod, gmsg, nowp.x, nowp.y, goal_x, goal_y)
        if replan_none_must_stop(path2):
            request_halt("REPLAN_FAILED -> SAFE_STOP")
            return None
        print(f"REPLAN_SUCCESS waypoints={len(path2)} length={path_len(path2):.3f}m", flush=True)
        for i, wp in enumerate(path2[:12]):
            print(f"  REPLAN_WP i={i} x={wp.x:.3f} y={wp.y:.3f}", flush=True)
        waypoints[:] = [(wp.x, wp.y) for wp in path2]
        wp_i["i"] = 1 if len(waypoints) > 1 else 0
        phase["name"] = "ALIGN"
        last_wi["i"] = -1
        path.poses.clear()
        path.header.stamp = node.get_clock().now().to_msg()
        for wp in path2:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = wp.x
            ps.pose.position.y = wp.y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        path_pub.publish(path)
        return [(wp.x, wp.y) for wp in path2[1:-1]]

    def drive(v, w):
        """Navigator callback: Guard only. Pulses sent from the avoid loop."""
        if halt_ev.is_set():
            return
        gv, gw, blocked = guard.guard_velocity(v, w)
        if blocked:
            stats["guard_hit"] = True
            stats["guard_reason"] = guard.last_blocked_reason

    ctrl = type("C", (), {"set_velocity": lambda self, v, w: None, "stop_motion": lambda self: None})()
    cfg = nav_mod.NavigateConfig(
        max_linear_m_s=V_CMD,
        max_angular_rad_s=AVOID_W_MAX,
        goal_tolerance_m=ARRIVE_M,
        stall_timeout_s=AVOID_TIMEOUT_S,
        replan_after_s=0.6,
        update_interval_s=0.05,
    )
    nav = nav_mod.Navigator(ctrl, guard=guard, config=cfg, drive=drive)
    nav.apply_external_pose(sx, sy, math.degrees(syaw))
    mids = [(wp.x, wp.y) for wp in wps[1:-1]]
    print(f"NAV_GOTO n_wp={len(wps)} n_mids={len(mids)} mids={mids}", flush=True)
    nav.goto(gx, gy, speed=V_CMD, waypoints=mids)

    ensure_package()
    from bunker_mini.teleop_tcp import DEFAULT_PORT, DEFAULT_TOKEN, TeleopTcpClient

    mode = can_control_mode()
    print(f"PREFLIGHT CAN 0x211 control_mode={mode}", flush=True)
    if mode != 1:
        print("FAIL PREFLIGHT control_mode!=1 不发运动", flush=True)
        nav.stop()
        ros_cleanup()
        return 8

    cli = None
    try:
        cli = TeleopTcpClient("127.0.0.1", DEFAULT_PORT, DEFAULT_TOKEN)
        cli.hello()
        cli.stick(0.0, 0.0)
        cli_box["cli"] = cli
    except Exception as exc:
        print(f"FAIL :9100 connect {exc}")
        nav.stop()
        ros_cleanup()
        return 6

    def send_pulse(v: float, w: float) -> None:
        if halt_ev.is_set():
            raise Halt(halt_reason["r"] or "halt")
        if cli is None:
            raise Halt(":9100 client missing")
        cli.cmd(
            {
                "action": "move",
                "v": float(v),
                "w": float(w),
                "duration": LIVE_GO_PULSE_S,
                "bypassGuard": True,
                "ts": int(time.time() * 1000),
            },
            timeout=0.8,
        )
        stats["sent_can"] = True

    def stop_motion_keep_session() -> None:
        """Stop chassis without estop so a later replan can resume."""
        if cli is None:
            return
        try:
            send_pulse(0.0, 0.0)
        except Exception:
            pass
        try:
            cli.stick(0.0, 0.0)
        except Exception:
            pass

    def _atexit_stop():
        halt_chassis(cli_box.get("cli"), "atexit")

    atexit.register(_atexit_stop)

    live["on"] = True
    live_t0 = time.monotonic()
    last_pulse = 0.0
    last_xy = (sx, sy)
    last_move_mono = time.monotonic()
    end_pose = (sx, sy, syaw)

    print("LIVE_AVOID_START", flush=True)
    print(f"goal=({gx:.3f},{gy:.3f}) forward={AVOID_GOAL_FWD_M:.2f}m", flush=True)
    print(
        f"pose=({sx:.3f},{sy:.3f},yaw_deg={math.degrees(syaw):.2f}) "
        f"pulse_s={LIVE_GO_PULSE_S:.2f} period_s={LIVE_GO_PERIOD_S:.2f}",
        flush=True,
    )

    try:
        while time.monotonic() - live_t0 < AVOID_TIMEOUT_S:
            snap, g_age = pull_obs()
            pose = state["pose"]
            grid = state["grid"]
            if pose is None or snap is None:
                raise Halt("pose lost")
            end_pose = pose
            gate = pose_gate(snap)
            nav.apply_external_pose(pose[0], pose[1], math.degrees(pose[2]))
            if grid is not None:
                ad = RosGridAdapter(grid)
                lidar.update_from_grid(ad, pose[0], pose[1], pose[2])
            if halt_ev.is_set():
                raise Halt(halt_reason["r"] or "halt")
            if g_age > GRID_STALE_S:
                raise Halt(f"grid stale age_s={g_age:.3f}")
            stats["max_age"] = max(stats["max_age"], float(snap.age_s))
            stats["n_pose_end"] = int(snap.n)
            remain = math.hypot(gx - pose[0], gy - pose[1])
            ax, ay, hy = segment_yaw_xy()
            _along_seg, cross = track_progress(ax, ay, hy, pose[0], pose[1])
            if abs(cross) > LATERAL_ABORT_M:
                raise Halt(f"lateral {cross:.3f} m off current segment")
            if len(waypoints) <= 2:
                dyaw = abs(wrap_deg(math.degrees(pose[2] - syaw)))
                if dyaw > YAW_ABORT_DEG:
                    raise Halt(f"yaw changed {dyaw:.1f} deg (rotation not allowed)")
            tx, ty, wi = current_target(pose[0], pose[1])
            if remain <= ARRIVE_M:
                stats["arrived"] = True
                stats["reason"] = f"arrived remain={remain:.3f}m"
                print(f"ARRIVED remaining={remain:.3f} wp={wi}", flush=True)
                break
            mode_now = mode_watch.mode()
            if mode_now is None and (time.monotonic() - live_t0) >= 1.0:
                raise Halt("no 0x211 control_mode")
            if mode_now is not None and mode_now != 1:
                print(f"CONTROL_MODE_ABORT actual={mode_now} expected=1", flush=True)
                raise Halt(f"CONTROL_MODE_ABORT actual={mode_now} expected=1")
            if gate == "HOLD":
                stop_motion_keep_session()
                time.sleep(0.02)
                continue
            if wi != last_wi["i"]:
                phase["name"] = "ALIGN"
                last_wi["i"] = wi
            v_cmd, w_cmd, err_deg, stage = align_drive_vw(
                pose[0], pose[1], pose[2], tx, ty, phase
            )
            gv, gw, blocked = guard.guard_velocity(v_cmd, w_cmd)
            if blocked:
                stats["guard_hit"] = True
                stats["guard_reason"] = guard.last_blocked_reason
                print(f"GUARD_BLOCK reason={guard.last_blocked_reason!r} remaining={remain:.3f}", flush=True)
                stop_motion_keep_session()
                mid = replanner(gx, gy)
                if mid is None or halt_ev.is_set():
                    print("REPLAN_FAILED -> SAFE_STOP", flush=True)
                    raise Halt("REPLAN_FAILED -> SAFE_STOP")
                print("REPLAN_SUCCESS resume ALIGN", flush=True)
                time.sleep(0.05)
                continue
            now = time.monotonic()
            if math.hypot(pose[0] - last_xy[0], pose[1] - last_xy[1]) >= 0.04:
                last_xy = (pose[0], pose[1])
                last_move_mono = now
            if now - last_move_mono >= 8.0 and gv > 0.03:
                raise Halt("no MOLA xy motion for 8s while commanding v — not faking pose")
            if now - last_pulse >= LIVE_GO_PERIOD_S:
                try:
                    send_pulse(gv, gw)
                except Halt:
                    raise
                except Exception as exc:
                    raise Halt(f":9100 move failed {exc}") from exc
                last_pulse = now
                stats["n_pulse"] += 1
                print(f"{stage} wp={wi} err={err_deg:.2f} v={gv:.3f} w={gw:.3f}", flush=True)
                print(
                    f" remaining={remain:.3f} target=({tx:.3f},{ty:.3f}) "
                    f"n_pulse={stats['n_pulse']} replan_n={stats['replan_n']} sent_can=1",
                    flush=True,
                )
            time.sleep(0.02)
        else:
            raise Halt("timeout")
    except Halt as h:
        stats["reason"] = h.reason
        print(f"LIVE_AVOID_ABORT reason={h.reason}", flush=True)
    except KeyboardInterrupt:
        stats["reason"] = "KeyboardInterrupt"
        print("LIVE_AVOID_ABORT reason=KeyboardInterrupt", flush=True)
    except Exception as exc:
        stats["reason"] = f"exception {exc}"
        print(f"LIVE_AVOID_ABORT reason=exception {exc}", flush=True)
    finally:
        live["on"] = False
        try:
            nav.stop()
        except Exception:
            pass
        print("HALT", flush=True)
        halt_chassis(cli, "live-avoid end")
        time.sleep(0.4)
        try:
            if cli is not None:
                cli.stick(0.0, 0.0)
        except Exception:
            pass
        vw = query_vw(cli)
        print(f"QUERY after stop v,w={vw} sent_can={int(stats['sent_can'])}", flush=True)
        try:
            if cli is not None:
                cli.close()
        except Exception:
            pass
        cli_box["cli"] = None
        ros_cleanup()

    ex, ey, eyaw = end_pose
    print("======== LIVE_AVOID REPORT ========", flush=True)
    print(f"start=({sx:.3f},{sy:.3f},{math.degrees(syaw):.2f}) goal=({gx:.3f},{gy:.3f})", flush=True)
    print(f"end=({ex:.3f},{ey:.3f},{math.degrees(eyaw):.2f})", flush=True)
    print(f"arrived={stats['arrived']} replan_n={stats['replan_n']} n_pulse={stats['n_pulse']}", flush=True)
    print(f"sent_can={stats['sent_can']} reason={stats['reason']!r}", flush=True)
    print(f"pose_source=MOLA /lidar_odometry/pose (not v*dt)", flush=True)
    return 0 if stats["arrived"] else 7


def main() -> int:
    ap = argparse.ArgumentParser(description="nav_demo real-grid avoid (independent of --live-go)")
    ap.add_argument(
        "--dry-only",
        action="store_true",
        help="A*/Guard/replan compute only; never TeleopTcpClient / CAN",
    )
    args = ap.parse_args()
    if args.dry_only:
        return dry_run_avoid()
    return run_live_avoid()


if __name__ == "__main__":
    sys.exit(main())
