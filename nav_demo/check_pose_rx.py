#!/usr/bin/env python3
"""No-chassis check: is 'MOLA pose stale' a publisher gap or Demo callback starvation?

Does not send stick / CAN / move. Only subscribes to /lidar_odometry/pose
(and one /nav_demo/obstacle_grid for adapter timing).
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from mola_pose import (  # noqa: E402
    MolaPoseWatch,
    NodeSpinThread,
    PoseSnap,
    classify_pose_rx,
    format_pose_rx,
    qos_mola_pose,
)


def _test_classify() -> bool:
    cases = [
        (0.10, 0.02, 0.05, "OK"),
        (0.50, 0.02, 0.05, "OK"),
        (0.501, 0.02, 0.05, "STALE_MOLA"),
        (0.571, 0.02, 0.05, "STALE_MOLA"),
        (0.10, 0.30, 0.05, "NO_SPIN"),
        (0.571, 0.30, 0.05, "NO_SPIN"),
        (0.10, 0.02, 0.60, "MAIN_STUCK"),
        (0.571, 0.02, 0.60, "STALE_MOLA"),
    ]
    ok = True
    print("CLASSIFY unit (no ROS)", flush=True)
    for age, spin, main, want in cases:
        got = classify_pose_rx(age, spin, main, stale_s=0.50)
        mark = "ok" if got == want else "FAIL"
        print(
            f"  {mark} age={age:.3f} spin={spin:.3f} main={main:.3f} -> {got} want={want}",
            flush=True,
        )
        if got != want:
            ok = False
    return ok


def _gaps(rx: list[float]) -> tuple[float, float, int]:
    if len(rx) < 2:
        return float("nan"), float("nan"), 0
    ds = [rx[i] - rx[i - 1] for i in range(1, len(rx))]
    over = sum(1 for d in ds if d > 0.50)
    return max(ds), sum(ds) / len(ds), over


def main() -> int:
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node

    from ros_grid import RosGridAdapter

    print("check_pose_rx: no stick, no CAN, no chassis", flush=True)
    classify_ok = _test_classify()
    rclpy.init()
    node = Node("nav_demo_check_pose_rx")
    watch = MolaPoseWatch(node)
    grid_box = {"msg": None, "n": 0}

    def on_grid(msg):
        grid_box["msg"] = msg
        grid_box["n"] += 1

    node.create_subscription(OccupancyGrid, "/nav_demo/obstacle_grid", on_grid, qos_mola_pose())

    # --- A: dedicated spinner, main thread idle (publisher truth) ---
    spinner = NodeSpinThread(node)
    spinner.start()
    t_end = time.monotonic() + 5.0
    skews: list[float] = []
    while time.monotonic() < t_end:
        snap = watch.snapshot()
        if snap is not None:
            skews.append(snap.stamp_skew_s)
        time.sleep(0.05)
    rx_a = watch.rx_monotonics()
    max_a, mean_a, over_a = _gaps(rx_a)
    snap = watch.snapshot()
    print(
        f"A publisher+bg_spin  n={len(rx_a)} max_gap={max_a:.3f}s mean={mean_a:.3f}s "
        f"gaps>0.50s={over_a} last_age={0 if snap is None else snap.age_s:.3f}s "
        f"stamp_skew_s={0 if not skews else sum(skews)/len(skews):.3f}",
        flush=True,
    )

    # adapter cost (main thread, callbacks still on spinner)
    adapter_s = float("nan")
    gmsg = grid_box["msg"]
    if gmsg is not None:
        t0 = time.monotonic()
        for _ in range(5):
            RosGridAdapter(gmsg)
        adapter_s = (time.monotonic() - t0) / 5.0
        print(
            f"RosGridAdapter mean={adapter_s*1000:.1f} ms  grid_n={grid_box['n']} "
            f"wh={gmsg.info.width}x{gmsg.info.height}",
            flush=True,
        )
    else:
        print("WARN no /nav_demo/obstacle_grid (adapter timing skipped)", flush=True)

    # Synthetic STALE_MOLA / MAIN_STUCK lines while spinner is actually live.
    fake_stale = PoseSnap(
        x=0.0,
        y=0.0,
        yaw=0.0,
        n=int(0 if snap is None else snap.n),
        rx_mono=time.monotonic() - 0.571,
        header_stamp_s=0.0,
        age_s=0.571,
        stamp_skew_s=0.0,
        last_gap_s=0.571,
    )
    line_stale = format_pose_rx(fake_stale, spinner, time.monotonic(), stale_s=0.50)
    line_main = format_pose_rx(snap, spinner, time.monotonic() - 0.80, stale_s=0.50)
    print(f"SYN {line_stale}", flush=True)
    print(f"SYN {line_main}", flush=True)
    syn_ok = ("state=STALE_MOLA" in line_stale) and ("state=MAIN_STUCK" in line_main)

    spinner.stop()

    # --- B: OLD live-loop pattern: 1x spin_once then block (pulse/adapter) ---
    # Recreate node so we do not mix executors.
    node.destroy_node()
    node = Node("nav_demo_check_pose_rx_b")
    rx_b: list[float] = []
    n_box = {"n": 0}

    def on_b(msg):
        n_box["n"] += 1
        rx_b.append(time.monotonic())

    from nav_msgs.msg import Odometry

    node.create_subscription(Odometry, "/lidar_odometry/pose", on_b, qos_mola_pose())
    t_end = time.monotonic() + 5.0
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.04)
        # reproduce first_live: full grid copy every tick + periodic TCP wait
        if gmsg is not None:
            RosGridAdapter(gmsg)
        time.sleep(0.18)  # send_pulse period; ACK is extra on top
    max_b, mean_b, over_b = _gaps(rx_b)
    print(
        f"B spin_once+adapter+sleep(0.18)  n={len(rx_b)} max_gap={max_b:.3f}s "
        f"mean={mean_b:.3f}s gaps>0.50s={over_b}",
        flush=True,
    )

    # --- C: worst-case TCP ACK 0.55s on same thread ---
    rx_c: list[float] = []

    def on_c(msg):
        rx_c.append(time.monotonic())

    node.destroy_node()
    node = Node("nav_demo_check_pose_rx_c")
    node.create_subscription(Odometry, "/lidar_odometry/pose", on_c, qos_mola_pose())
    t_end = time.monotonic() + 4.0
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.04)
        time.sleep(0.55)  # < cmd timeout 0.8, > POSE_STALE_S 0.50
    max_c, mean_c, over_c = _gaps(rx_c)
    print(
        f"C spin_once+sleep(0.55)  n={len(rx_c)} max_gap={max_c:.3f}s "
        f"mean={mean_c:.3f}s gaps>0.50s={over_c}",
        flush=True,
    )

    # --- D: 0.55s block with spinner (short contrast to C) ---
    node.destroy_node()
    node = Node("nav_demo_check_pose_rx_d")
    watch_d = MolaPoseWatch(node)
    spinner = NodeSpinThread(node)
    spinner.start()
    t_end = time.monotonic() + 4.0
    while time.monotonic() < t_end:
        time.sleep(0.55)
    spinner.stop()
    rx_d = watch_d.rx_monotonics()
    max_d, mean_d, over_d = _gaps(rx_d)
    print(
        f"D bg_spin+sleep(0.55)  n={len(rx_d)} max_gap={max_d:.3f}s "
        f"mean={mean_d:.3f}s gaps>0.50s={over_d}",
        flush=True,
    )

    # --- F: pause spinner → NO_SPIN (n_pose frozen); restart → recover ---
    node.destroy_node()
    node = Node("nav_demo_check_pose_rx_f")
    watch_f = MolaPoseWatch(node)
    spinner = NodeSpinThread(node)
    spinner.start()
    t_warm = time.monotonic() + 8.0
    while time.monotonic() < t_warm:
        snap_w = watch_f.snapshot()
        if snap_w is not None and snap_w.n >= 3 and snap_w.age_s < 0.30:
            break
        time.sleep(0.05)
    n_f0 = 0 if watch_f.snapshot() is None else watch_f.snapshot().n
    last_f_log = {"t": 0.0, "s": ""}
    spinner.stop()
    seen_no_spin = False
    t_pause = time.monotonic()
    while time.monotonic() - t_pause < 0.40:
        snap_f = watch_f.snapshot()
        age = float("inf") if snap_f is None else snap_f.age_s
        st = classify_pose_rx(age, spinner.spin_age_s(), 0.0, stale_s=0.50)
        now = time.monotonic()
        if now - last_f_log["t"] >= 0.20 or st != last_f_log["s"]:
            print(format_pose_rx(snap_f, spinner, now, stale_s=0.50), flush=True)
            last_f_log["t"] = now
            last_f_log["s"] = st
        if st == "NO_SPIN":
            seen_no_spin = True
        time.sleep(0.05)
    n_frozen = 0 if watch_f.snapshot() is None else watch_f.snapshot().n
    spinner.start()
    recovered = False
    t_rec = time.monotonic()
    n_f1 = n_frozen
    while time.monotonic() - t_rec < 8.0:
        snap_f = watch_f.snapshot()
        now = time.monotonic()
        st = "NONE" if snap_f is None else classify_pose_rx(
            snap_f.age_s, spinner.spin_age_s(), 0.0, stale_s=0.50
        )
        if now - last_f_log["t"] >= 0.25 or st != last_f_log["s"]:
            print(format_pose_rx(snap_f, spinner, now, stale_s=0.50), flush=True)
            last_f_log["t"] = now
            last_f_log["s"] = st
        if snap_f is not None and snap_f.n > n_frozen and snap_f.age_s < 0.50:
            recovered = True
            n_f1 = snap_f.n
            break
        time.sleep(0.05)
    if watch_f.snapshot() is not None:
        n_f1 = watch_f.snapshot().n
    spinner.stop()
    frozen_ok = n_f0 >= 3 and n_frozen == n_f0
    print(
        f"F spinner_pause NO_SPIN={int(seen_no_spin)} n_frozen_ok={int(frozen_ok)} "
        f"recovered={int(recovered)} n_pose {n_f0}->{n_frozen}->{n_f1}",
        flush=True,
    )
    f_ok = seen_no_spin and frozen_ok and recovered

    # --- E: 15s NodeSpinThread + main-thread 0.8s waits (send_pulse ACK cap) ---
    # No TCP, no stick: sleep(0.8) is the same scheduler stall as cmd timeout=0.8.
    node.destroy_node()
    node = Node("nav_demo_check_pose_rx_e")
    watch_e = MolaPoseWatch(node)
    spinner = NodeSpinThread(node)
    spinner.start()
    ages: list[float] = []
    n_samples: list[int] = []
    sample_stop = {"on": False}

    def _sample_age() -> None:
        while not sample_stop["on"]:
            snap_e = watch_e.snapshot()
            if snap_e is not None:
                ages.append(snap_e.age_s)
                n_samples.append(snap_e.n)
            time.sleep(0.02)

    sampler = threading.Thread(target=_sample_age, name="age-sample", daemon=True)
    sampler.start()
    t_end = time.monotonic() + 15.0
    block_n_ok = 0
    block_n_fail = 0
    while time.monotonic() < t_end:
        n_before = 0 if watch_e.snapshot() is None else watch_e.snapshot().n
        time.sleep(0.80)  # longest send_pulse ACK wait
        n_after = 0 if watch_e.snapshot() is None else watch_e.snapshot().n
        if n_after > n_before:
            block_n_ok += 1
        else:
            block_n_fail += 1
    sample_stop["on"] = True
    sampler.join(timeout=1.0)
    spinner.stop()
    rx_e = watch_e.rx_monotonics()
    max_e, mean_e, over_e = _gaps(rx_e)
    max_age = max(ages) if ages else float("nan")
    n0 = n_samples[0] if n_samples else 0
    n1 = n_samples[-1] if n_samples else 0
    print(
        f"E 15s bg_spin+sleep(0.80)  n={len(rx_e)} n_pose {n0}->{n1} "
        f"max_gap={max_e:.3f}s mean={mean_e:.3f}s gaps>0.50s={over_e} "
        f"max_age_s={max_age:.3f} blocks_got_pose={block_n_ok} blocks_no_pose={block_n_fail}",
        flush=True,
    )

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    pub_ok = over_a == 0 and (max_a == max_a) and max_a < 0.50
    starve = over_c > 0 or (max_c == max_c and max_c > 0.50)
    # D/E may see MOLA's own ~0.5s rest gaps; pass if spinner still received while main slept.
    fixed_d = len(rx_d) >= 15
    n_grew = n1 > n0 and len(rx_e) >= 15
    tcp_ok = block_n_ok >= 10 and block_n_fail <= 2
    fixed_e = n_grew and tcp_ok
    print(
        f"VERDICT classify_ok={int(classify_ok)} syn_ok={int(syn_ok)} "
        f"publisher_gaps_over_0.5={over_a} "
        f"old_loop_can_starve={int(starve)} "
        f"bg_spin_survives_0.55_block={int(fixed_d)} "
        f"bg_spin_survives_0.80_block_15s={int(fixed_e)} "
        f"spinner_pause_recover={int(f_ok)}",
        flush=True,
    )
    if starve and not pub_ok:
        print("note: publisher itself had >0.5s gaps at rest; keep stale=0.50", flush=True)
    return 0 if classify_ok and syn_ok and starve and fixed_d and fixed_e and f_ok else 1


if __name__ == "__main__":
    sys.exit(main())
