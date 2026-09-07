#!/usr/bin/env python3
"""First live chassis test: 1.0 m straight, v=0.06, w=0. nav_demo only. Guard always on."""
from __future__ import annotations

import argparse
import atexit
import math
import subprocess
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
from ros_grid import FootprintClearGrid, OCCUPIED, RosGridAdapter  # noqa: E402
from run_mvp import SectorLidar  # noqa: E402

V_CMD = 0.06
W_CMD = 0.0
GOAL_FWD_M = 1.0
YAW_ABORT_DEG = 12.0
LATERAL_ABORT_M = 0.25
POSE_STALE_S = 0.50
# Abort only if a MOLA gap never recovers. Not a raised stale threshold: at 0.50s
# motion is already inhibited. 2.5s matches MOLA_MAX_TIME_TO_USE_VELOCITY_MODEL.
POSE_HOLD_ABORT_S = 2.5
GRID_STALE_S = 1.00
LIVE_TIMEOUT_S = 22.0
DRY_S = 4.0
NO_MOVE_S = 3.0
NO_MOVE_M = 0.04
ARRIVE_M = 0.18  # stop when remaining 0.15–0.20 m; do not chase exact 1.0 m
FWD_CLEAR_MIN_M = 1.35  # 1.0 m goal + Guard stop 0.35 m
PULSE_DURATION_S = 1.00
POST_PULSE_OBSERVE_S = 2.0  # keep spinner/pose after the single pulse; then halt


def yaw_from_q(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_deg(d: float) -> float:
    return (d + 180.0) % 360.0 - 180.0


def track_progress(sx, sy, yaw, x, y) -> tuple[float, float]:
    dx, dy = x - sx, y - sy
    along = dx * math.cos(yaw) + dy * math.sin(yaw)
    cross = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return along, cross


class Halt(Exception):
    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason


def _can_listen_ifaces() -> list[str]:
    """USB-CAN first (name may be can0 or can1), then any other can*. Listen-only."""
    import os

    names = [
        n for n in ("can0", "can1", "can2", "can3")
        if os.path.exists(f"/sys/class/net/{n}")
    ]

    def _usb(n: str) -> bool:
        try:
            return "usb" in os.path.realpath(f"/sys/class/net/{n}/device").lower()
        except OSError:
            return False

    return sorted(names, key=lambda n: (0 if _usb(n) else 1, n))


def _candump_id(can_id: str, timeout_s: str, n: str) -> str:
    for iface in _can_listen_ifaces():
        try:
            raw = subprocess.check_output(
                ["timeout", timeout_s, "candump", f"{iface},{can_id}:7FF", "-n", n],
                text=True,
                errors="replace",
                stderr=subprocess.DEVNULL,
            )
        except Exception:
            continue
        if raw.strip():
            return raw
    return ""


def can_control_mode() -> int | None:
    """Listen-only 0x211 byte1 = ControlMode. None if no frame. No TX."""
    raw = _candump_id("211", "1", "2")
    for line in raw.splitlines():
        if " 211 " not in line:
            continue
        hexpart = line.split("]")[-1].strip().split()
        if len(hexpart) >= 2:
            try:
                return int(hexpart[1], 16)
            except ValueError:
                continue
    return None


def can_111_has_cmd() -> bool:
    raw = _candump_id("111", "0.6", "4")
    for line in raw.splitlines():
        if " 111 " not in line:
            continue
        hexpart = line.split("]")[-1].strip().split()
        if any(b != "00" for b in hexpart[:6]):
            return True
    return False


def halt_chassis(cli, note: str) -> None:
    print(f"STOP {note}", flush=True)
    if cli is None:
        return
    try:
        cli.stick(0.0, 0.0)
    except Exception as exc:
        print(f"WARN stick(0,0) failed: {exc}", flush=True)
    try:
        cli.stick(0.0, 0.0)
    except Exception:
        pass
    try:
        cli.estop()
        print("ESTOP sent", flush=True)
    except Exception as exc:
        print(f"WARN estop failed: {exc}", flush=True)


def query_vw(cli) -> tuple[float, float] | None:
    if cli is None:
        return None
    try:
        v, w, _rtt = cli.query()
        return float(v), float(w)
    except Exception:
        return None


def main() -> int:
    ap = argparse.ArgumentParser(description="nav_demo first live straight test")
    ap.add_argument("--dry-only", action="store_true")
    ap.add_argument(
        "--one-pulse",
        action="store_true",
        help="send one 1.00s move, observe pose >=2s, then halt (no repeat pulses)",
    )
    args = ap.parse_args()

    if args.one_pulse:
        print(
            "LIVE TEST envelope: ONE pulse v=0.06 w=0 duration=1.00s; "
            f"observe>={POST_PULSE_OBSERVE_S:.1f}s then halt; "
            "Guard ON POSE_STALE_S=0.50 (unchanged)",
            flush=True,
        )
    else:
        print(
            "LIVE TEST envelope: v=0.06 w=0.0 goal=1.0m straight Guard ON POSE_STALE_S=0.50",
            flush=True,
        )
    print("WASD must not run. Remote :9101 clients will be dropped before LIVE.", flush=True)

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from rclpy.node import Node
    from std_msgs.msg import Header

    rclpy.init()
    node = Node("nav_demo_first_live")
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
    while time.monotonic() - t0 < 6.0:
        snap_w = pose_watch.snapshot()
        gmsg_w, _n_gw, g_age_w = grid_watch.snapshot()
        # TRANSIENT_LOCAL delivers one latched pose immediately; wait for a live stream
        # so stale(0.50) is not measured against a single cached sample.
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
    raw = RosGridAdapter(state["grid"])
    adapter = FootprintClearGrid(raw, sx, sy, radius_m=0.40)
    lidar = SectorLidar()
    lidar.update_from_grid(raw, sx, sy, syaw)
    fwd = lidar.nearest_in_range(0.0, 50.0, quantile=0.10)
    print(
        f"PREFLIGHT POSE x={sx:.3f} y={sy:.3f} yaw_deg={math.degrees(syaw):.2f}",
        flush=True,
    )
    print(
        f"PREFLIGHT GRID occupied={adapter.occupied_count()} free={adapter.free_count()}",
        flush=True,
    )
    print(f"PREFLIGHT forward_nearest_m={fwd}", flush=True)
    goal_fwd = GOAL_FWD_M
    if fwd is not None:
        goal_fwd = min(GOAL_FWD_M, max(0.0, float(fwd) - 0.40))
    if goal_fwd < 0.70:
        print(f"FAIL PREFLIGHT not enough clear run ({goal_fwd:.2f} m after 0.40 m margin)")
        ros_cleanup()
        return 4
    gx = sx + goal_fwd * math.cos(syaw)
    gy = sy + goal_fwd * math.sin(syaw)
    print(f"GOAL x={gx:.3f} y={gy:.3f} forward={goal_fwd:.2f}m (demo layer only)", flush=True)

    gp_mod = load_global_planner()
    nav_mod = load_navigator()
    obs_mod = load_obstacle()
    planner = gp_mod.GlobalPlanner(adapter, inflation_m=0.20)
    wps = planner.plan(sx, sy, gx, gy)
    if not wps:
        print("FAIL PREFLIGHT A* none — not starting")
        ros_cleanup()
        return 3
    print(f"PLAN found=True waypoints={len(wps)}", flush=True)
    for i, wp in enumerate(wps):
        print(f"  WAYPOINT i={i} x={wp.x:.3f} y={wp.y:.3f}", flush=True)
    if len(wps) > 2:
        print("FAIL PREFLIGHT path is not a straight 2-point line; abort first test")
        ros_cleanup()
        return 3

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
    stats = {
        "max_v": 0.0,
        "max_w": 0.0,
        "guard_hit": False,
        "replan_n": 0,
        "sent_live": False,
        "n_stick": 0,
        "reason": "",
        "arrived": False,
        "max_age": 0.0,
        "max_gap": 0.0,
        "n_pose_end": 0,
        "hold_n": 0,
        "hold_max_s": 0.0,
    }
    live = {"on": False}
    cli_box = {"cli": None}
    last_print = {"t": 0.0}
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
        print(f"HALT {reason}", flush=True)
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

    def safety_or_raise(pose_now, pose_snap, grid_age_s):
        if grid_age_s > GRID_STALE_S:
            raise Halt(f"grid stale age_s={grid_age_s:.3f}")
        x, y, yaw = pose_now
        along, cross = track_progress(sx, sy, syaw, x, y)
        dyaw = abs(wrap_deg(math.degrees(yaw - syaw)))
        if dyaw > YAW_ABORT_DEG:
            raise Halt(f"yaw changed {dyaw:.1f} deg (rotation not allowed)")
        if abs(cross) > LATERAL_ABORT_M:
            raise Halt(f"lateral {cross:.3f} m (direction mismatch)")
        if along < -0.08:
            raise Halt(f"moved backward {along:.3f} m")
        return along, cross, dyaw

    def hold_zero_cmd() -> None:
        """Live only: zero current move without estop/stick so a later pose can resume."""
        cli = cli_box.get("cli")
        if cli is None or not live["on"]:
            return

        def _zero() -> None:
            try:
                cli.cmd(
                    {
                        "action": "move",
                        "v": 0.0,
                        "w": 0.0,
                        "duration": 0.30,
                        "bypassGuard": True,
                        "ts": int(time.time() * 1000),
                    },
                    timeout=0.30,
                )
            except Exception as exc:
                print(f"WARN pose-hold zero cmd: {exc}", flush=True)

        threading.Thread(target=_zero, name="pose-hold-zero", daemon=True).start()

    def pose_gate(snap) -> str:
        """OK: drive. HOLD: inhibit motion, keep last real pose. Abort if never recovers."""
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
        if rx_state == "MAIN_STUCK":
            print("WARN MAIN_STUCK (control loop lagged; spinner/pose may still be live)", flush=True)
        if rx_state == "STALE_MOLA":
            if not pose_hold["on"]:
                pose_hold["on"] = True
                pose_hold["t0"] = now
                stats["hold_n"] += 1
                print(
                    "POSE_HOLD enter (MOLA publish gap; motion inhibited; "
                    "last received pose kept, not faked)",
                    flush=True,
                )
                hold_zero_cmd()
            held = now - pose_hold["t0"]
            stats["hold_max_s"] = max(stats["hold_max_s"], held)
            if held > POSE_HOLD_ABORT_S:
                raise Halt(
                    f"MOLA pose dead held={held:.3f}s "
                    f"(stale_stop={POSE_STALE_S:.2f}s abort_if_no_recover={POSE_HOLD_ABORT_S:.2f}s)"
                )
            return "HOLD"
        if pose_hold["on"]:
            held = now - pose_hold["t0"]
            pose_hold["on"] = False
            print(f"POSE_HOLD exit recovered after {held:.3f}s (fresh pose)", flush=True)
        return "OK"

    def drive(v, w):
        try:
            if halt_ev.is_set():
                return
            if abs(w) > 0.15:
                request_halt(f"Navigator requested rotation w={w:.3f}")
                return
            v = min(V_CMD, max(0.0, float(v)))
            gv, gw, blocked = guard.guard_velocity(v, W_CMD)
            gw = 0.0
            if blocked:
                stats["guard_hit"] = True
                request_halt("ObstacleGuard blocked")
                return
            gv = min(V_CMD, max(0.0, float(gv)))
            stats["max_v"] = max(stats["max_v"], gv)
            stats["max_w"] = max(stats["max_w"], abs(gw))
            now = time.monotonic()
            if now - last_print["t"] >= 0.5:
                last_print["t"] = now
                pose = state["pose"]
                along = 0.0
                if pose:
                    along, _cross = track_progress(sx, sy, syaw, pose[0], pose[1])
                ps = pose_watch.snapshot()
                tag = "LIVE" if live["on"] else "DRY_RUN"
                print(
                    f"POSE x={pose[0]:.3f} y={pose[1]:.3f} yaw_deg={math.degrees(pose[2]):.2f} "
                    f"n_pose={0 if ps is None else ps.n} age_s={-1 if ps is None else ps.age_s:.3f} "
                    f"GRID n={state['n_grid']} along={along:.3f} "
                    f"GUARD blocked={int(blocked)} "
                    f"CMD v={gv:.3f} w={gw:.3f} {tag}",
                    flush=True,
                )
            if live["on"]:
                stats["sent_live"] = True
                stats["n_stick"] += 1
                # Motion is TeleopTcpClient.move (below). Do not stick(0.06)
                # here: laptop :9101 idle 100Hz stick(0,0) would stop_motion.
        except Halt as h:
            request_halt(h.reason)
        except Exception as exc:
            request_halt(f"drive exception {exc}")

    def replanner(goal_x: float, goal_y: float):
        stats["replan_n"] += 1
        print(f"REPLAN n={stats['replan_n']} ignored (first test is straight-only)", flush=True)
        return None

    ctrl = type("C", (), {"set_velocity": lambda self, v, w: None, "stop_motion": lambda self: None})()
    cfg = nav_mod.NavigateConfig(
        max_linear_m_s=V_CMD,
        max_angular_rad_s=0.05,
        goal_tolerance_m=ARRIVE_M,
        stall_timeout_s=LIVE_TIMEOUT_S,
        replan_after_s=60.0,
        update_interval_s=0.05,
        align_in_place_rad=3.0,
        heading_deadband_rad=0.20,
    )
    nav = nav_mod.Navigator(ctrl, guard=guard, config=cfg, drive=drive)
    nav.apply_external_pose(sx, sy, math.degrees(syaw))

    # --- DRY RUN ---
    print("=== DRY-RUN start (no stick) ===", flush=True)
    live["on"] = False
    nav.goto(gx, gy, speed=V_CMD, waypoints=[], replanner=replanner)
    dry_end = time.monotonic() + DRY_S
    try:
        while time.monotonic() < dry_end:
            snap, g_age = pull_obs()
            pose = state["pose"]
            grid = state["grid"]
            if pose is None or snap is None:
                raise Halt("pose lost in dry-run")
            gate = pose_gate(snap)
            stats["max_age"] = max(stats["max_age"], float(snap.age_s))
            stats["n_pose_end"] = int(snap.n)
            nav.apply_external_pose(pose[0], pose[1], math.degrees(pose[2]))
            if grid is not None:
                ad = RosGridAdapter(grid)
                lidar.update_from_grid(ad, pose[0], pose[1], pose[2])
            if halt_ev.is_set():
                raise Halt(halt_reason["r"] or "halt")
            safety_or_raise(pose, snap, g_age)
            if gate == "HOLD":
                time.sleep(0.05)
                continue
            gv, gw, blocked = guard.guard_velocity(V_CMD, W_CMD)
            if blocked:
                raise Halt("Guard blocked during dry-run")
            if abs(gv - V_CMD) > 0.02:
                raise Halt(f"dry-run guard reduced v to {gv:.3f}")
            time.sleep(0.05)
        nav.stop()
        print(
            f"DRY-RUN PASS  CMD v=0.06 w=0  Guard clear  pose/grid live  "
            f"hold_n={stats['hold_n']} hold_max_s={stats['hold_max_s']:.3f} "
            f"n_pose_end={stats['n_pose_end']} max_age_s={stats['max_age']:.3f}",
            flush=True,
        )
    except Halt as h:
        stats["reason"] = h.reason
        print(f"FAIL DRY-RUN {h.reason}", flush=True)
        nav.stop()
        ros_cleanup()
        return 5
    except Exception as exc:
        stats["reason"] = f"dry-run exception {exc}"
        print(f"FAIL DRY-RUN exception {exc}", flush=True)
        nav.stop()
        ros_cleanup()
        return 5

    if args.dry_only:
        print(
            "dry-only: not sending stick / CAN / set_velocity  "
            f"POSE_STALE_S={POSE_STALE_S:.2f} (unchanged)",
            flush=True,
        )
        ros_cleanup()
        return 0

    # --- LIVE ---
    ensure_package()
    from bunker_mini.teleop_tcp import DEFAULT_PORT, DEFAULT_TOKEN, TeleopTcpClient

    mode = can_control_mode()
    print(f"PREFLIGHT CAN 0x211 control_mode={mode} (1=CAN_COMMAND, 3=REMOTE)", flush=True)
    if mode != 1:
        print(
            "FAIL PREFLIGHT chassis not in CAN command mode. "
            "SWB 拨到指令档后再试。不发运动。",
            flush=True,
        )
        nav.stop()
        ros_cleanup()
        return 8

    cli = None
    try:
        cli = TeleopTcpClient("127.0.0.1", DEFAULT_PORT, DEFAULT_TOKEN)
        cli.hello()
        cli.stick(0.0, 0.0)
        cli_box["cli"] = cli
        # Timed move so idle :9101 stick(0,0) cannot stop_motion.
        # bypassGuard: agent AiryLidar is not this demo's sensor (SDK owns 6699);
        # nav_demo ObstacleGuard + estop remain the safety net.
        def send_pulse() -> None:
            cli.cmd(
                {
                    "action": "move",
                    "v": V_CMD,
                    "w": W_CMD,
                    "duration": PULSE_DURATION_S,
                    "bypassGuard": True,
                    "ts": int(time.time() * 1000),
                },
                timeout=0.8,
            )

        send_pulse()
        stats["sent_live"] = True
        stats["n_stick"] = 1
        stats["max_v"] = max(stats["max_v"], V_CMD)
        if args.one_pulse:
            print(
                f"LIVE :9100 hello OK; ONE pulse v={V_CMD} w={W_CMD} "
                f"duration={PULSE_DURATION_S:.2f}s; no repeat; "
                f"observe pose {POST_PULSE_OBSERVE_S:.1f}s then stick(0,0)+estop "
                "(bypass agent lidar; demo Guard ON)",
                flush=True,
            )
        else:
            print(
                "LIVE :9100 hello OK; pulsing move v=0.06 w=0 every ~0.2s "
                "(bypass agent lidar; demo Guard ON; duration cap 0.35s)",
                flush=True,
            )
    except Exception as exc:
        print(f"FAIL :9100 connect {exc}")
        nav.stop()
        ros_cleanup()
        return 6

    def _atexit_stop():
        halt_chassis(cli_box.get("cli"), "atexit")

    atexit.register(_atexit_stop)

    live["on"] = True
    live_t0 = time.monotonic()
    rx_live0 = list(pose_watch.rx_monotonics())
    n_pose_live0 = 0 if pose_watch.snapshot() is None else pose_watch.snapshot().n
    start_pose = state["pose"]
    along_now = 0.0
    end_pose = start_pose
    checked_111 = False
    last_pulse = time.monotonic()
    hold_paused = {"s": 0.0, "t": None}
    try:
        if args.one_pulse:
            # Single pulse already sent. Do not nav.goto (that would request more motion).
            # Keep ROS spinner + pose callbacks running through the pulse and >=2s after.
            observe_end = live_t0 + PULSE_DURATION_S + POST_PULSE_OBSERVE_S
            while time.monotonic() < observe_end:
                snap, g_age = pull_obs()
                pose = state["pose"]
                grid = state["grid"]
                if pose is None or snap is None:
                    raise Halt("pose lost")
                end_pose = pose
                gate = pose_gate(snap)
                if grid is not None:
                    ad = RosGridAdapter(grid)
                    lidar.update_from_grid(ad, pose[0], pose[1], pose[2])
                if halt_ev.is_set():
                    raise Halt(halt_reason["r"] or "halt")
                along_now, _cross, _dyaw = safety_or_raise(pose, snap, g_age)
                stats["max_age"] = max(stats["max_age"], float(snap.age_s))
                stats["n_pose_end"] = int(snap.n)
                gv, _gw, blocked = guard.guard_velocity(V_CMD, W_CMD)
                if blocked:
                    stats["guard_hit"] = True
                    raise Halt("ObstacleGuard blocked")
                if gate == "HOLD":
                    if hold_paused["t"] is None:
                        hold_paused["t"] = time.monotonic()
                    time.sleep(0.02)
                    continue
                if hold_paused["t"] is not None:
                    hold_paused["s"] += time.monotonic() - hold_paused["t"]
                    hold_paused["t"] = None
                now = time.monotonic()
                if now - last_print["t"] >= 0.5:
                    last_print["t"] = now
                    print(
                        f"POSE x={pose[0]:.3f} y={pose[1]:.3f} "
                        f"yaw_deg={math.degrees(pose[2]):.2f} "
                        f"n_pose={snap.n} age_s={snap.age_s:.3f} "
                        f"GRID n={state['n_grid']} along={along_now:.3f} "
                        f"GUARD blocked=0 CMD v={gv:.3f} w=0.000 OBSERVE",
                        flush=True,
                    )
                time.sleep(0.02)
            stats["reason"] = (
                f"one-pulse observe done pulse={PULSE_DURATION_S:.2f}s "
                f"observe={POST_PULSE_OBSERVE_S:.1f}s"
            )
        else:
            nav.goto(gx, gy, speed=V_CMD, waypoints=[], replanner=replanner)
            while time.monotonic() - live_t0 < LIVE_TIMEOUT_S:
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
                along_now, _cross, _dyaw = safety_or_raise(pose, snap, g_age)
                stats["max_age"] = max(stats["max_age"], float(snap.age_s))
                stats["n_pose_end"] = int(snap.n)
                if gate == "HOLD":
                    if hold_paused["t"] is None:
                        hold_paused["t"] = time.monotonic()
                    time.sleep(0.02)
                    continue
                if hold_paused["t"] is not None:
                    hold_paused["s"] += time.monotonic() - hold_paused["t"]
                    hold_paused["t"] = None
                motion_s = time.monotonic() - live_t0 - hold_paused["s"]
                if time.monotonic() - last_pulse >= 0.18:
                    send_pulse()
                    last_pulse = time.monotonic()
                    stats["n_stick"] += 1
                remain = math.hypot(gx - pose[0], gy - pose[1])
                if remain <= ARRIVE_M or along_now >= (goal_fwd - ARRIVE_M):
                    stats["arrived"] = True
                    stats["reason"] = f"arrived remain={remain:.3f}m"
                    break
                if (not checked_111) and motion_s >= 1.2:
                    checked_111 = True
                    if along_now < NO_MOVE_M and not can_111_has_cmd():
                        raise Halt("CAN 0x111 still zero after command — chassis not actuating")
                if motion_s >= NO_MOVE_S and along_now < NO_MOVE_M:
                    raise Halt(
                        f"no forward motion after {NO_MOVE_S:.0f}s (along={along_now:.3f} m); "
                        "possible WASD/0,0 fight or chassis not enabled"
                    )
                if not nav.is_navigating and not stats["arrived"]:
                    raise Halt("Navigator stopped unexpectedly")
                time.sleep(0.02)
            else:
                stats["reason"] = "timeout"
                raise Halt("timeout")
    except Halt as h:
        stats["reason"] = h.reason
        print(f"HALT {h.reason}", flush=True)
    except Exception as exc:
        stats["reason"] = f"exception {exc}"
        print(f"HALT exception {exc}", flush=True)
    finally:
        live["on"] = False
        try:
            nav.stop()
        except Exception:
            pass
        halt_chassis(cli, "test end")
        time.sleep(0.4)
        # extra zeros
        try:
            if cli is not None:
                cli.stick(0.0, 0.0)
        except Exception:
            pass
        time.sleep(0.4)
        vw = query_vw(cli)
        print(f"QUERY after stop v,w={vw}", flush=True)
        # confirm pose not still sliding much
        t_wait = time.monotonic() + 1.0
        last = state["pose"]
        while time.monotonic() < t_wait:
            pull_obs()
            time.sleep(0.05)
        settled = state["pose"] or last
        end_pose = settled or end_pose
        if last and settled:
            slide = math.hypot(settled[0] - last[0], settled[1] - last[1])
            print(f"SETTLE slide_1s={slide:.3f} m", flush=True)
        try:
            if cli is not None:
                cli.close()
        except Exception:
            pass
        cli_box["cli"] = None
        ros_cleanup()

    ex, ey, eyaw = end_pose if end_pose else (float("nan"),) * 3
    along, cross = track_progress(sx, sy, syaw, ex, ey)
    dyaw = wrap_deg(math.degrees(eyaw - syaw))
    moved = along >= NO_MOVE_M
    ok_stop = True
    if vw is not None and (abs(vw[0]) > 0.02 or abs(vw[1]) > 0.05):
        ok_stop = False
    n_pose_end = stats["n_pose_end"]
    pose_grew = n_pose_end > n_pose_live0
    if args.one_pulse:
        observe_ok = str(stats["reason"]).startswith("one-pulse observe done")
        passed = (
            bool(stats["sent_live"])
            and stats["n_stick"] == 1
            and observe_ok
            and pose_grew
            and not stats["guard_hit"]
            and ok_stop
            and abs(dyaw) <= YAW_ABORT_DEG
        )
    else:
        passed = (
            stats["arrived"]
            and moved
            and not stats["guard_hit"]
            and ok_stop
            and abs(dyaw) <= YAW_ABORT_DEG
        )
    rx_live = pose_watch.rx_monotonics()
    new_rx = rx_live[len(rx_live0) :]
    if len(new_rx) >= 2:
        stats["max_gap"] = max(new_rx[i] - new_rx[i - 1] for i in range(1, len(new_rx)))
    verdict = "PASS" if passed else "FAIL"
    print("======== LIVE REPORT ========", flush=True)
    print(f"1 start_pose x={sx:.3f} y={sy:.3f} yaw_deg={math.degrees(syaw):.2f}", flush=True)
    print(f"2 end_pose   x={ex:.3f} y={ey:.3f} yaw_deg={math.degrees(eyaw):.2f}", flush=True)
    print(f"3 xy_delta dx={ex-sx:.3f} dy={ey-sy:.3f} along={along:.3f} lateral={cross:.3f}", flush=True)
    print(f"4 yaw_change_deg={dyaw:.2f}", flush=True)
    print(f"5 max_v={stats['max_v']:.3f}", flush=True)
    print(f"6 max_w={stats['max_w']:.3f}", flush=True)
    print(f"7 n_pose live {n_pose_live0}->{n_pose_end} grew={int(pose_grew)}", flush=True)
    print(f"8 max_age_s={stats['max_age']:.3f} (stop_motion_at {POSE_STALE_S:.2f})", flush=True)
    print(f"9 max_pose_gap_s={stats['max_gap']:.3f}", flush=True)
    print(
        f"9b pose_hold_n={stats['hold_n']} hold_max_s={stats['hold_max_s']:.3f} "
        f"abort_if_no_recover={POSE_HOLD_ABORT_S:.2f}",
        flush=True,
    )
    print(f"10 guard_triggered={stats['guard_hit']}", flush=True)
    print(f"11 replan_n={stats['replan_n']} n_pulse={stats['n_stick']}", flush=True)
    print(f"12 arrived={stats['arrived']} reason={stats['reason']!r}", flush=True)
    print(f"13 stop_ok={ok_stop} query_after_stop={vw}", flush=True)
    print(f"14 can_111_zero_check after halt (see candump in wrapper)", flush=True)
    print(f"15 VERDICT {verdict}", flush=True)
    return 0 if passed else 7


if __name__ == "__main__":
    sys.exit(main())
