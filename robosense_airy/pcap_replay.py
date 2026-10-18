"""
RoboSense Airy — offline PCAP replay + WRS RSView-style 3D viewer.

Includes a simple forward danger-zone obstacle detector for dorm / pure-software
simulation (terminal alert + red highlight in 3D view).

Run (default uses bundled indoor capture)::

    python wrs/drivers/devices/robosense_airy/pcap_replay.py

Custom file / speed / loop::

    python wrs/drivers/devices/robosense_airy/pcap_replay.py \\
        --pcap airy_6x12_indoor(1).pcap --speed 1.5 --no-loop
"""
from __future__ import annotations

import argparse
from pathlib import Path

from wrs import rm, wd
from wrs.drivers.devices.robosense_airy.airy_driver import RoboSenseAiryPcap
from wrs.drivers.devices.robosense_airy.obstacle import (
    DangerZone,
    add_obstacle_cli_args,
    danger_zone_from_args,
    make_danger_zone_wireframe,
    print_obstacle_banner,
    process_obstacle_frame,
)
from wrs.drivers.devices.robosense_airy.pcap_reader import (
    DEFAULT_PCAP,
    Backend,
    _resolve_backend,
    load_msop_packets,
)
from wrs.drivers.devices.robosense_airy.viz import (
    DEFAULT_MAX_POINTS,
    DEFAULT_POINT_SIZE_PX,
    EGO_FILTER_RADIUS,
    IntensityPointCloudRenderer,
    apply_rsview_scene,
    make_ground_grid,
)


def run_viewer(pcap_path: str | Path,
               loop: bool = True,
               speed: float = 1.0,
               max_points: int = DEFAULT_MAX_POINTS,
               point_size: float = DEFAULT_POINT_SIZE_PX,
               min_distance: float = 0.2,
               max_distance: float = 60.0,
               verbose: bool = False,
               backend: Backend = "auto",
               danger_zone: DangerZone | None = None,
               obstacle_detection: bool = True) -> None:
    pcap_path = Path(pcap_path)
    if not pcap_path.is_file():
        raise FileNotFoundError(f"PCAP not found: {pcap_path}")

    zone = danger_zone or DangerZone()
    chosen = _resolve_backend(backend)
    packets = load_msop_packets(pcap_path, backend=backend)
    duration = packets[-1][0] - packets[0][0] if len(packets) > 1 else 0.0

    print("=" * 60)
    title = "PCAP Offline Replay + 避障仿真" if obstacle_detection else "PCAP Offline Replay"
    print(f"  RoboSense Airy — {title}")
    print("=" * 60)
    print(f"  PCAP file    : {pcap_path}")
    print(f"  Backend      : {chosen}  (no pypcap)")
    print(f"  MSOP packets : {len(packets):,}")
    print(f"  Duration     : {duration:.1f} s")
    print(f"  Speed        : {speed}×")
    print(f"  Loop         : {'on' if loop else 'off'}")
    print(f"  Max points   : {max_points:,}")
    print(f"  Point size   : {point_size} px")
    if obstacle_detection:
        print_obstacle_banner(zone)
    print("=" * 60)

    base = wd.World(
        cam_pos=rm.vec(0, -20, 12),
        lookat_pos=rm.vec(0, 0, 0.5),
        w=1280, h=720,
    )
    scene_title = "RoboSense Airy — PCAP Replay + Obstacle" if obstacle_detection else None
    apply_rsview_scene(base, title=scene_title)
    make_ground_grid().reparentTo(base.render)
    if obstacle_detection:
        make_danger_zone_wireframe(zone).reparentTo(base.render)

    lidar = RoboSenseAiryPcap(
        pcap_path=str(pcap_path),
        loop=loop,
        speed=speed,
        min_distance=min_distance,
        max_distance=max_distance,
        dense_points=True,
        verbose=verbose,
        display_max_points=max_points,
        ego_filter_radius=EGO_FILTER_RADIUS,
        pcap_backend=backend,
    )

    renderer = IntensityPointCloudRenderer(
        base, max_points=max_points, point_size_px=point_size)
    state = {"last_seq": 0, "rendered": 0, "was_alert": False, "danger_count": 0}

    def update(lidar, renderer, zone, obstacle_on, state, task):
        pcd, intensity, seq = lidar.poll_frame(
            last_seq=state["last_seq"], remove_nan=False)
        if pcd is None or len(pcd) == 0:
            return task.cont

        state["last_seq"] = seq
        if obstacle_on:
            count, alert = process_obstacle_frame(
                lidar, pcd, intensity, renderer, zone, state)
        else:
            count, alert = 0, False
            renderer.update(pcd, intensity)

        state["rendered"] += 1
        if state["rendered"] % 10 == 0 or state["rendered"] == 1:
            raw = lidar.last_raw_point_count
            if obstacle_on:
                flag = "⚠ OBSTACLE" if alert else "  safe"
                print(f"  frame #{state['rendered']:4d}  |  "
                      f"{raw:>7,} raw → {len(pcd):>6,} show  |  "
                      f"ROI={count:4d}  {flag}")
            else:
                print(f"  frame #{state['rendered']:4d}  |  "
                      f"{raw:>7,} raw → {len(pcd):>6,} show")
        return task.cont

    base.taskMgr.add(update, "airy_pcap_update",
                     extraArgs=[lidar, renderer, zone, obstacle_detection, state],
                     appendTask=True)

    try:
        base.run()
    except KeyboardInterrupt:
        pass
    finally:
        lidar.stop()
        renderer.detach()
        print("Done.")


def main():
    parser = argparse.ArgumentParser(
        description="RoboSense Airy PCAP replay + forward obstacle sim")
    parser.add_argument(
        "--pcap", type=str, default=str(DEFAULT_PCAP),
        help=f"PCAP file path (default: {DEFAULT_PCAP.name})")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--no-loop", action="store_true")
    parser.add_argument("--max-points", type=int, default=DEFAULT_MAX_POINTS)
    parser.add_argument("--point-size", type=float, default=DEFAULT_POINT_SIZE_PX)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--min-distance", type=float, default=0.2)
    parser.add_argument("--max-distance", type=float, default=60.0)
    parser.add_argument(
        "--backend", choices=("auto", "scapy", "binary"), default="auto")
    add_obstacle_cli_args(parser)
    args = parser.parse_args()

    zone = danger_zone_from_args(args)
    obstacle_on = not args.no_obstacle

    run_viewer(
        pcap_path=args.pcap,
        loop=not args.no_loop,
        speed=args.speed,
        max_points=args.max_points,
        point_size=args.point_size,
        min_distance=args.min_distance,
        max_distance=args.max_distance,
        verbose=args.verbose,
        backend=args.backend,
        danger_zone=zone,
        obstacle_detection=obstacle_on,
    )


if __name__ == "__main__":
    main()
