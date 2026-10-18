#!/usr/bin/env python3
"""nav_demo live-go: closed-loop 1.0 m straight at v=0.06, w=0. Independent of --live one-pulse.

Uses real /lidar_odometry/pose only (never integrates v*dt). Pulses TeleopTcpClient
move duration<=0.30 s ~every 0.18 s until ARRIVE, then halt_chassis.
Does not change run_first_live.py safety thresholds.
"""
from __future__ import annotations

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
from ros_grid import FootprintClearGrid, RosGridAdapter  # noqa: E402
from run_first_live import (  # noqa: E402
    ARRIVE_M,
    DRY_S,
    GOAL_FWD_M,
    GRID_STALE_S,
    LATERAL_ABORT_M,
    LIVE_TIMEOUT_S,
    NO_MOVE_M,
    NO_MOVE_S,
    POSE_HOLD_ABORT_S,
    POSE_STALE_S,
    V_CMD,
    W_CMD,
    YAW_ABORT_DEG,
    Halt,
    _can_listen_ifaces,
    can_control_mode,
    halt_chassis,
    query_vw,
    track_progress,
    wrap_deg,
)
from run_mvp import SectorLidar  # noqa: E402

# Pulse timing for live-go only. Do not change run_first_live.PULSE_DURATION_S (one-pulse=1.00).
LIVE_GO_PULSE_S = 0.30
LIVE_GO_PERIOD_S = 0.18
LIVE_GO_W = 0.0  # first autonomous test: no yaw command


class ControlModeWatch:
    """Listen-only 0x211. No CAN TX."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._mode: int | None = None
        self._stop = threading.Event()
        self._proc: subprocess.Popen | None = None
        self._th: threading.Thread | None = None

    def start(self) -> None:
        self._th = threading.Thread(target=self._run, name="live-go-211", daemon=True)
        self._th.start()

    def mode(self) -> int | None:
        with self._lock:
            return self._mode

    def stop(self) -> None:
        self._stop.set()
        proc = self._proc
        if proc is not None and proc.poll() is None:
            try:
                proc.kill()
            except Exception:
                pass
        th = self._th
        if th is not None and th.is_alive() and threading.current_thread() is not th:
            th.join(timeout=1.0)

    def _run(self) -> None:
        ifaces = _can_listen_ifaces()
        if not ifaces:
            return
        try:
            self._proc = subprocess.Popen(
                ["candump", f"{ifaces[0]},211:7FF"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
            )
        except Exception:
            return
        assert self._proc.stdout is not None
        for line in self._proc.stdout:
            if self._stop.is_set():
                break
            if " 211 " not in line:
                continue
            hexpart = line.split("]")[-1].strip().split()
            if len(hexpart) < 2:
                continue
            try:
                mode = int(hexpart[1], 16)
            except ValueError:
                continue
            with self._lock:
                self._mode = mode


def run_live_go() -> int:
    print(
        "LIVE_GO envelope: v=0.06 w=0 goal=1.0m straight ARRIVE_M=0.18 "
        f"POSE_STALE_S={POSE_STALE_S:.2f} pulse={LIVE_GO_PULSE_S:.2f}s "
        f"period={LIVE_GO_PERIOD_S:.2f}s Guard require_sensor=True",
        flush=True,
    )
    print("WASD / :9101 must not run. Independent of --live one-pulse.", flush=True)

    import atexit

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from rclpy.node import Node
    from std_msgs.msg import Header

    rclpy.init()
    node = Node("nav_demo_live_go")
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
        print("FAIL PREFLIGHT path is not a straight 2-point line; abort first live-go")
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
        "n_pulse": 0,
        "reason": "",
        "arrived": False,
        "max_age": 0.0,
        "n_pose_end": 0,
        "hold_n": 0,
        "hold_max_s": 0.0,
        "guard_reason": "",
    }
    live = {"on": False}
    cli_box: dict = {"cli": None}
    halt_ev = threading.Event()
    halt_reason = {"r": ""}
    pose_hold = {"on": False, "t0": 0.0}
    main_tick = {"t": time.monotonic()}
    last_rx_log = {"t": 0.0, "state": ""}
    last_nav = {"v": 0.0, "w": 0.0}

    def request_halt(reason: str) -> None:
        if halt_ev.is_set():
            return
        halt_reason["r"] = reason
        halt_ev.set()
        print(f"LIVE_GO_ABORT reason={reason}", flush=True)
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

    def safety_or_raise(pose_now, grid_age_s):
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
                print(
                    "POSE_HOLD enter (MOLA publish gap; motion inhibited; "
                    "last received pose kept, not faked)",
                    flush=True,
                )
                cli = cli_box.get("cli")
                if live["on"] and cli is not None:

                    def _zero() -> None:
                        try:
                            cli.cmd(
                                {
                                    "action": "move",
                                    "v": 0.0,
                                    "w": 0.0,
                                    "duration": LIVE_GO_PULSE_S,
                                    "bypassGuard": True,
                                    "ts": int(time.time() * 1000),
                                },
                                timeout=0.30,
                            )
                        except Exception as exc:
                            print(f"WARN pose-hold zero cmd: {exc}", flush=True)

                    threading.Thread(target=_zero, name="live-go-hold-zero", daemon=True).start()
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
        """Navigator output: Guard check only. Pulses are sent from the live-go loop."""
        try:
            if halt_ev.is_set():
                return
            if abs(w) > 0.15:
                request_halt(f"Navigator requested rotation w={w:.3f}")
                return
            v = min(V_CMD, max(0.0, float(v)))
            gv, gw, blocked = guard.guard_velocity(v, LIVE_GO_W)
            last_nav["v"], last_nav["w"] = gv, 0.0
            if blocked:
                stats["guard_hit"] = True
                stats["guard_reason"] = guard.last_blocked_reason
                request_halt(f"ObstacleGuard blocked reason={guard.last_blocked_reason!r}")
                return
            stats["max_v"] = max(stats["max_v"], gv)
            stats["max_w"] = max(stats["max_w"], abs(gw))
        except Halt as h:
            request_halt(h.reason)
        except Exception as exc:
            request_halt(f"drive exception {exc}")

    def replanner(_goal_x: float, _goal_y: float):
        print("REPLAN ignored (live-go is straight-only)", flush=True)
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

    print("=== DRY-RUN start (no stick / no move) ===", flush=True)
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
            nav.apply_external_pose(pose[0], pose[1], math.degrees(pose[2]))
            if grid is not None:
                ad = RosGridAdapter(grid)
                lidar.update_from_grid(ad, pose[0], pose[1], pose[2])
            if halt_ev.is_set():
                raise Halt(halt_reason["r"] or "halt")
            safety_or_raise(pose, g_age)
            if gate == "HOLD":
                time.sleep(0.05)
                continue
            gv, _gw, blocked = guard.guard_velocity(V_CMD, LIVE_GO_W)
            if blocked:
                raise Halt(f"Guard blocked during dry-run reason={guard.last_blocked_reason!r}")
            if abs(gv - V_CMD) > 0.02:
                raise Halt(f"dry-run guard reduced v to {gv:.3f}")
            time.sleep(0.05)
        nav.stop()
        print("DRY-RUN PASS  Guard clear  pose/grid live  (no CAN)", flush=True)
    except Halt as h:
        print(f"FAIL DRY-RUN {h.reason}", flush=True)
        nav.stop()
        ros_cleanup()
        return 5
    except Exception as exc:
        print(f"FAIL DRY-RUN exception {exc}", flush=True)
        nav.stop()
        ros_cleanup()
        return 5

    ensure_package()
    from bunker_mini.teleop_tcp import DEFAULT_PORT, DEFAULT_TOKEN, TeleopTcpClient

    mode = can_control_mode()
    print(f"PREFLIGHT CAN 0x211 control_mode={mode} (1=CAN_COMMAND, 3=REMOTE)", flush=True)
    if mode != 1:
        print("FAIL PREFLIGHT chassis not in CAN command mode. 不发运动。", flush=True)
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

    def send_pulse(v: float) -> None:
        if cli is None:
            raise Halt(":9100 client missing")
        cli.cmd(
            {
                "action": "move",
                "v": float(v),
                "w": LIVE_GO_W,
                "duration": LIVE_GO_PULSE_S,
                "bypassGuard": True,
                "ts": int(time.time() * 1000),
            },
            timeout=0.8,
        )

    def _atexit_stop():
        halt_chassis(cli_box.get("cli"), "atexit")

    atexit.register(_atexit_stop)

    live["on"] = True
    live_t0 = time.monotonic()
    n_pose_live0 = 0 if pose_watch.snapshot() is None else pose_watch.snapshot().n
    start_pose = state["pose"]
    end_pose = start_pose
    last_pulse = 0.0
    last_pose_log = {"t": 0.0}
    hold_paused = {"s": 0.0, "t": None}

    print("LIVE_GO_START", flush=True)
    print(f"goal=({gx:.3f},{gy:.3f}) forward={goal_fwd:.3f}m", flush=True)
    print(
        f"pose=({sx:.3f},{sy:.3f},yaw_deg={math.degrees(syaw):.2f}) "
        f"pulse_s={LIVE_GO_PULSE_S:.2f} period_s={LIVE_GO_PERIOD_S:.2f} v={V_CMD:.3f} w={LIVE_GO_W:.3f}",
        flush=True,
    )

    try:
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
            along_now, _cross, _dyaw = safety_or_raise(pose, g_age)
            stats["max_age"] = max(stats["max_age"], float(snap.age_s))
            stats["n_pose_end"] = int(snap.n)
            remain = math.hypot(gx - pose[0], gy - pose[1])
            now_log = time.monotonic()
            if now_log - last_pose_log["t"] >= 0.25:
                last_pose_log["t"] = now_log
                print(
                    f"LIVE_POSE x={pose[0]:.3f} y={pose[1]:.3f} "
                    f"yaw={math.degrees(pose[2]):.2f} remaining={remain:.3f} "
                    f"along={along_now:.3f} n_pose={snap.n} age_s={snap.age_s:.3f}",
                    flush=True,
                )
            if remain <= ARRIVE_M or along_now >= (goal_fwd - ARRIVE_M):
                stats["arrived"] = True
                stats["reason"] = f"arrived remain={remain:.3f}m along={along_now:.3f}m"
                print(f"LIVE_GO_ARRIVED remaining={remain:.3f} along={along_now:.3f}", flush=True)
                break
            mode_now = mode_watch.mode()
            if mode_now is None and (time.monotonic() - live_t0) >= 1.0:
                raise Halt("no 0x211 control_mode (listen-only watch)")
            if mode_now is not None and mode_now != 1:
                raise Halt(f"control_mode={mode_now} (want 1)")
            if gate == "HOLD":
                if hold_paused["t"] is None:
                    hold_paused["t"] = time.monotonic()
                time.sleep(0.02)
                continue
            if hold_paused["t"] is not None:
                hold_paused["s"] += time.monotonic() - hold_paused["t"]
                hold_paused["t"] = None
            gv, _gw, blocked = guard.guard_velocity(V_CMD, LIVE_GO_W)
            if blocked:
                stats["guard_hit"] = True
                stats["guard_reason"] = guard.last_blocked_reason
                raise Halt(f"ObstacleGuard blocked reason={guard.last_blocked_reason!r}")
            v_cmd = min(V_CMD, max(0.0, float(gv)))
            now = time.monotonic()
            motion_s = now - live_t0 - hold_paused["s"]
            if now - last_pulse >= LIVE_GO_PERIOD_S:
                try:
                    send_pulse(v_cmd)
                except Halt:
                    raise
                except Exception as exc:
                    raise Halt(f":9100 move failed {exc}") from exc
                last_pulse = now
                stats["n_pulse"] += 1
                print(
                    f"LIVE_PULSE remaining={remain:.3f} v={v_cmd:.3f} w={LIVE_GO_W:.3f} "
                    f"guard=ALLOW n_pulse={stats['n_pulse']} duration={LIVE_GO_PULSE_S:.2f}",
                    flush=True,
                )
            if motion_s >= NO_MOVE_S and along_now < NO_MOVE_M:
                raise Halt(
                    f"no forward motion after {NO_MOVE_S:.0f}s (along={along_now:.3f} m); "
                    "MOLA pose did not move — not faking distance"
                )
            time.sleep(0.02)
        else:
            raise Halt("timeout")
    except Halt as h:
        stats["reason"] = h.reason
        print(f"LIVE_GO_ABORT reason={h.reason}", flush=True)
    except KeyboardInterrupt:
        stats["reason"] = "KeyboardInterrupt"
        print("LIVE_GO_ABORT reason=KeyboardInterrupt", flush=True)
    except Exception as exc:
        stats["reason"] = f"exception {exc}"
        print(f"LIVE_GO_ABORT reason=exception {exc}", flush=True)
    finally:
        live["on"] = False
        try:
            nav.stop()
        except Exception:
            pass
        print("LIVE_GO_HALT", flush=True)
        halt_chassis(cli, "live-go end")
        time.sleep(0.4)
        try:
            if cli is not None:
                cli.stick(0.0, 0.0)
        except Exception:
            pass
        time.sleep(0.4)
        vw = query_vw(cli)
        print(f"QUERY after stop v,w={vw}", flush=True)
        t_wait = time.monotonic() + 1.0
        last = state["pose"]
        while time.monotonic() < t_wait:
            pull_obs()
            time.sleep(0.05)
        settled = state["pose"] or last
        end_pose = settled or end_pose
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
    n_pose_end = stats["n_pose_end"]
    print("======== LIVE_GO REPORT ========", flush=True)
    print(f"1 start_pose x={sx:.3f} y={sy:.3f} yaw_deg={math.degrees(syaw):.2f}", flush=True)
    print(f"2 end_pose   x={ex:.3f} y={ey:.3f} yaw_deg={math.degrees(eyaw):.2f}", flush=True)
    print(f"3 along={along:.3f} lateral={cross:.3f} yaw_change_deg={dyaw:.2f}", flush=True)
    print(f"4 n_pose live {n_pose_live0}->{n_pose_end}", flush=True)
    print(f"5 max_age_s={stats['max_age']:.3f} POSE_STALE_S={POSE_STALE_S:.2f}", flush=True)
    print(f"6 n_pulse={stats['n_pulse']} pulse_s={LIVE_GO_PULSE_S:.2f} period_s={LIVE_GO_PERIOD_S:.2f}", flush=True)
    print(f"7 guard_triggered={stats['guard_hit']} reason={stats['guard_reason']!r}", flush=True)
    print(f"8 arrived={stats['arrived']} reason={stats['reason']!r}", flush=True)
    print(f"9 pose_source=MOLA /lidar_odometry/pose (not v*dt)", flush=True)
    return 0 if stats["arrived"] and not stats["guard_hit"] else 7


def main() -> int:
    return run_live_go()


if __name__ == "__main__":
    sys.exit(main())
