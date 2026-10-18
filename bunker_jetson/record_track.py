#!/usr/bin/env python3
"""Record a BUNKER MINI 2.0 trajectory — "轨迹录制".

While this script runs, manually drive the chassis (via remote control or
CAN commands). The script samples odometry and motion at 100 ms intervals
and saves them as a track file for later replay.

Usage::

    # candleLight (auto-detect):
    python examples/record_track.py --name warehouse_loop

    # candleLight (manual):
    python examples/record_track.py --name warehouse_loop --interface gs_usb --channel 0030001F4148570C20343133:0

    # Serial CAN:
    python examples/record_track.py --name warehouse_loop --interface slcan --channel COM3

    # Record for 30 seconds then auto-stop:
    python examples/record_track.py --name delivery_A --duration 30
    # Press Enter to stop manually.
"""

from __future__ import annotations

import argparse
import sys
import time

from _bootstrap import ensure_project_root

ensure_project_root()

from bunker_mini import BunkerMiniController
from bunker_mini.can_util import CanConfigError, add_can_cli_args, format_device_list, print_can_config_help
from bunker_mini.tracker import Track, TrackPlayer, TrackRecorder


def main() -> int:
    parser = argparse.ArgumentParser(description="Record a BUNKER MINI 2.0 trajectory")
    add_can_cli_args(parser)
    parser.add_argument("--name", "-n", default="", help="Track name (default: auto-generated)")
    parser.add_argument(
        "--duration", "-d", type=float, default=0,
        help="Recording duration in seconds (0 = manual stop via Enter)",
    )
    parser.add_argument(
        "--track-dir", default="./tracks",
        help="Directory to save track files (default: ./tracks)",
    )
    args = parser.parse_args()

    if args.list_devices:
        print(format_device_list())
        return 0

    try:
        controller = BunkerMiniController(channel=args.channel, interface=args.interface)
    except CanConfigError as exc:
        print(f"CAN 配置错误:\n{exc}", file=sys.stderr)
        print(file=sys.stderr)
        print_can_config_help()
        return 2

    controller.start()
    try:
        # Wait for chassis feedback
        for _ in range(200):
            if controller.latest_status is not None:
                break
            time.sleep(0.05)
        else:
            print("错误: 未收到底盘状态 (0x211)，请检查 CAN 接线和波特率(500K)")
            return 1

        recorder = TrackRecorder(controller, track_dir=args.track_dir)
        recorder.start(args.name)

        interrupted = False
        try:
            if args.duration > 0:
                print(f"录制中... ({args.duration} 秒后自动停止)")
                time.sleep(args.duration)
            else:
                print("录制中... 请手动驾驶小车走完路线，然后按 Enter 停止录制")
                input()
        except KeyboardInterrupt:
            interrupted = True
            print("\n录制被中断，尝试保存已录制的部分轨迹...")

        if recorder.is_recording:
            try:
                track = recorder.stop()
                player = TrackPlayer(controller, track_dir=args.track_dir)
                path = player.save_track(track)
                print(f"轨迹已保存: {path}")
                print(f"  名称: {track.name}")
                print(f"  时长: {track.total_duration_s:.1f} s")
                print(f"  航点数: {len(track.waypoints)}")
                iface_part = f"--interface {args.interface} " if args.interface else ""
                ch_part = f"--channel {args.channel} " if args.channel else ""
                print(f"回放命令: python examples/play_track.py --name {track.name} {iface_part}{ch_part}")
            except RuntimeError as e:
                print(f"录制数据不足，未保存: {e}")

        return 130 if interrupted else 0
    except KeyboardInterrupt:
        print("\n已中断")
        return 130
    finally:
        controller.stop()


if __name__ == "__main__":
    sys.exit(main())
