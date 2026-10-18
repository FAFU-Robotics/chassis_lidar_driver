#!/usr/bin/env python3
"""Replay a recorded BUNKER MINI 2.0 trajectory — "轨迹回放".

Feeds the recorded velocity commands back to the chassis at the original
timestamps so it autonomously retraces the path.

Usage::

    # Play by track name (candleLight, auto-detect):
    python examples/play_track.py --name warehouse_loop

    # Play with explicit device:
    python examples/play_track.py --name warehouse_loop --interface gs_usb --channel 0030001F4148570C20343133:0

    # Play by absolute path:
    python examples/play_track.py --file ./tracks/warehouse_loop.json

    # Play 3 loops:
    python examples/play_track.py --name warehouse_loop --loop 3

    # List available tracks:
    python examples/play_track.py --list-tracks
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

from _bootstrap import ensure_project_root

ensure_project_root()

from bunker_mini import BunkerMiniController, ControlMode, VehicleState
from bunker_mini.can_util import CanConfigError, add_can_cli_args, format_device_list, print_can_config_help
from bunker_mini.tracker import Track, TrackPlayer

DEFAULT_TRACK_DIR = "./tracks"


def list_tracks(track_dir: str) -> list[str]:
    p = Path(track_dir)
    if not p.exists():
        return []
    return sorted(f.stem for f in p.glob("*.json"))


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay a BUNKER MINI 2.0 trajectory")
    add_can_cli_args(parser)
    parser.add_argument("--name", "-n", default="", help="Track name (looks in --track-dir)")
    parser.add_argument("--file", "-f", default="", help="Absolute path to a track JSON file")
    parser.add_argument("--track-dir", default=DEFAULT_TRACK_DIR, help=f"Track directory (default: {DEFAULT_TRACK_DIR})")
    parser.add_argument("--list-tracks", action="store_true", help="List saved tracks and exit")
    parser.add_argument("--loop", type=int, default=1, help="Number of replay loops (default: 1)")
    args = parser.parse_args()

    if args.list_devices:
        print(format_device_list())
        return 0

    if args.list_tracks:
        tracks = list_tracks(args.track_dir)
        if tracks:
            print("已保存的轨迹:")
            for t in tracks:
                print(f"  {t}")
        else:
            print(f"(目录 {args.track_dir} 中无轨迹文件)")
        return 0

    if not args.name and not args.file:
        tracks = list_tracks(track_dir)
        if tracks:
            print("已保存的轨迹（回放时用 --name <轨迹名> 选择）:")
            for t in tracks:
                print(f"  {t}")
            print()
            print(f"回放示例: python examples\\play_track.py --name {tracks[0]}")
        else:
            print(f"(目录 {track_dir} 中无轨迹文件，请先录制)")
        return 1

    # Find track
    track_dir = args.track_dir
    if args.file:
        path = Path(args.file)
        if not path.exists():
            print(f"错误: 轨迹文件不存在: {args.file}", file=sys.stderr)
            return 1
        with open(path, "r", encoding="utf-8") as f:
            track = Track.from_json(f.read())
    else:
        # Look up by name
        path = Path(track_dir) / f"{args.name}.json"
        if not path.exists():
            # Try exact path
            path = Path(args.name)
        if not path.exists():
            print(f"错误: 找不到轨迹 '{args.name}'", file=sys.stderr)
            existing = list_tracks(track_dir)
            if existing:
                print(f"可用轨迹: {', '.join(existing)}", file=sys.stderr)
            return 1
        with open(path, "r", encoding="utf-8") as f:
            track = Track.from_json(f.read())

    # Init CAN controller
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
            status = controller.latest_status
            if status is not None:
                break
            time.sleep(0.05)
        else:
            print("错误: 未收到底盘状态 (0x211)，请检查 CAN 接线")
            return 1

        status = controller.latest_status
        if status and status.vehicle_state != VehicleState.NORMAL:
            print(f"底盘状态异常: {status.vehicle_state.name}，请先清除故障")
            return 1
        if status and status.control_mode == ControlMode.REMOTE_CONTROL:
            print("遥控器处于控制模式且优先级更高，请先切换遥控器到指令模式")
            return 1

        print("切换底盘到 CAN 指令模式...")
        controller.enable_can_control()
        time.sleep(0.3)

        player = TrackPlayer(controller, track_dir=track_dir)

        print(f"轨迹: {track.name}")
        print(f"时长: {track.total_duration_s:.1f} s")
        print(f"航点: {len(track.waypoints)}")
        print(f"回放次数: {args.loop}")
        print()

        for i in range(args.loop):
            if args.loop > 1:
                print(f"--- 第 {i+1}/{args.loop} 次回放 ---")
            print("回放开始，按 Ctrl+C 中断...")
            try:
                player.play(track)
            except KeyboardInterrupt:
                print("\n回放已中断")
                player.stop()
                break

        return 0
    except KeyboardInterrupt:
        print("\n已退出")
        return 130
    finally:
        controller.stop()


if __name__ == "__main__":
    sys.exit(main())
