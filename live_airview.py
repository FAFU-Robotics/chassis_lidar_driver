#!/usr/bin/env python3
"""
RoboSense Airy — live UDP AirView (real-time 3D point cloud).

Uses only gated full frames published by RoboSenseAiry (no PCAP, no chassis).

Usage::

    python3 live_airview.py
    python3 live_airview.py --port 6699 --max-points 80000 --verbose
"""
from __future__ import annotations

import argparse

from wrs import rm, wd

from robosense_airy.airy_driver import RoboSenseAiry
from robosense_airy.viz import (
    DEFAULT_MAX_POINTS,
    DEFAULT_POINT_SIZE_PX,
    EGO_FILTER_RADIUS,
    IntensityPointCloudRenderer,
    apply_rsview_scene,
    make_ground_grid,
)


def run_live_viewer(msop_port: int = 6699,
                    max_points: int = 80_000,
                    point_size: float = DEFAULT_POINT_SIZE_PX,
                    min_distance: float = 0.2,
                    max_distance: float = 60.0,
                    verbose: bool = False) -> None:
    print("=== RoboSense Airy Live AirView ===")
    print(f"UDP port     : {msop_port}")
    print(f"max points   : {max_points:,}")
    print(f"point size   : {point_size}")
    print(f"min distance : {min_distance} m")
    print(f"max distance : {max_distance} m")
    print("Source       : RoboSenseAiry (live UDP, gated full frames only)")
    print("Ctrl+C to quit.")

    base = wd.World(
        cam_pos=rm.vec(0, -20, 12),
        lookat_pos=rm.vec(0, 0, 0.5),
        w=1280, h=720,
    )
    apply_rsview_scene(base, title="RoboSense Airy — Live AirView")
    make_ground_grid().reparentTo(base.render)

    # Full-frame gate lives inside RoboSenseAiry / _AiryMsopDecoder.
    # This viewer only displays frames that the driver has already accepted.
    lidar = RoboSenseAiry(
        msop_port=msop_port,
        min_distance=min_distance,
        max_distance=max_distance,
        dense_points=True,
        split_angle=0.0,
        verbose=verbose,
        display_max_points=max_points,
        ego_filter_radius=EGO_FILTER_RADIUS,
    )

    renderer = IntensityPointCloudRenderer(
        base, max_points=max_points, point_size_px=point_size)
    state = {"last_seq": 0, "rendered": 0}

    def update(lidar, renderer, state, task):
        pcd, intensity, seq = lidar.poll_frame(
            last_seq=state["last_seq"], remove_nan=False)
        if pcd is None or len(pcd) == 0:
            return task.cont

        # New accepted full-frame only (seq advances solely on driver publish).
        state["last_seq"] = seq
        renderer.update(pcd, intensity)

        state["rendered"] += 1
        if state["rendered"] % 10 == 0 or state["rendered"] == 1:
            raw = lidar.last_raw_point_count
            print(
                f"frame #{state['rendered']:4d} | "
                f"raw {raw:>7,} | "
                f"show {len(pcd):>6,} | "
                f"seq {seq}"
            )
        return task.cont

    base.taskMgr.add(
        update, "airy_live_update",
        extraArgs=[lidar, renderer, state],
        appendTask=True,
    )

    try:
        base.run()
    except KeyboardInterrupt:
        pass
    finally:
        lidar.stop()
        renderer.detach()
        print("Stopped.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="RoboSense Airy live UDP 3D AirView (no chassis / no PCAP)")
    parser.add_argument("--port", type=int, default=6699,
                        help="MSOP UDP port (default: 6699)")
    parser.add_argument("--max-points", type=int, default=80_000,
                        help="Display point budget (default: 80000)")
    parser.add_argument("--point-size", type=float, default=DEFAULT_POINT_SIZE_PX,
                        help=f"Point size in px (default: {DEFAULT_POINT_SIZE_PX})")
    parser.add_argument("--min-distance", type=float, default=0.2,
                        help="Min range filter in metres (default: 0.2)")
    parser.add_argument("--max-distance", type=float, default=60.0,
                        help="Max range filter in metres (default: 60.0)")
    parser.add_argument("--verbose", action="store_true",
                        help="Print per-frame accept/drop logs from the driver")
    args = parser.parse_args()

    run_live_viewer(
        msop_port=args.port,
        max_points=args.max_points,
        point_size=args.point_size,
        min_distance=args.min_distance,
        max_distance=args.max_distance,
        verbose=args.verbose,
    )


if __name__ == "__main__":
    main()
