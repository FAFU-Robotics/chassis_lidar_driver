#!/usr/bin/env python3
"""方式三：基于 RoboSense Airy 点云的反应式避障演示（Jetson / Linux）。

原理：不做建图、不做全局路径规划，直接把 360° 点云按方位角分成 N 个扇区，
统计每扇区点数；「正前方/左右侧」点密集 → 判断有障碍 → 主动寻缝转向、
倒车、或穿缝缓行。基础巡航速度已调低到 0.10 m/s（安全性优先）。

用法（两个 USB-CAN 都插好，雷达网线接 Jetson 板载网口）::

    bash bringup_gs_usb_can.sh          # 把 USB-CAN 拉到 socketcan
    python3 run_obstacle_avoidance.py

    # 手动指定 CAN 通道（接口名重启后可能变化，不要硬编码 can1）
    python3 run_obstacle_avoidance.py --can-channel can0

    # 只想看点云、不动车（雷达自检）
    python3 run_obstacle_avoidance.py --no-obstacle

    # 调低巡航速度 / 调整避障敏感度
    python3 run_obstacle_avoidance.py --cruise 0.08 --front-thresh 50

键盘：Ctrl+C 急停退出。
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Optional

from _bootstrap import ensure_project_root

ensure_project_root()

from bunker_mini.can_util import ensure_socketcan_interface  # noqa: E402
from robosense_airy.airy_driver import RoboSenseAiry  # noqa: E402
from robosense_airy.reactive_avoid import (  # noqa: E402
    CRUISE_VX,
    DECISION_CMDS,
    ObstacleAvoidance,
    points_to_sector_counts,
)

CRUISE_LINEAR = 0.10  # 基础巡航速度（原 0.15，已调低以提高安全性）


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="RoboSense Airy 反应式避障（方式三，Jetson/SocketCAN）")
    p.add_argument("--msop-port", type=int, default=6699, help="雷达 MSOP UDP 端口")
    p.add_argument("--can-channel", default=None,
                   help="SocketCAN 通道（默认自动检测 USB-CAN，勿硬编码）")
    p.add_argument("--cruise", type=float, default=CRUISE_LINEAR,
                   help="巡航线速度 m/s")
    p.add_argument("--n-sectors", type=int, default=24,
                   help="扇区数（默认 24，每扇区 15°）")
    p.add_argument("--range", type=float, default=1.5,
                   help="避障统计半径 m（默认 1.5）")
    p.add_argument("--front-thresh", type=int, default=40,
                   help="正前方扇区触发避障的点数阈值")
    p.add_argument("--side-thresh", type=int, default=25,
                   help="侧方扇区触发避障的点数阈值")
    p.add_argument("--no-obstacle", action="store_true",
                   help="只接收点云，不下发运动指令（自检用）")
    p.add_argument("--no-viz", action="store_true",
                   help="禁用终端 ASCII 扇区视图（打印频率下降）")
    return p


def _fmt_decision(decision: str) -> str:
    names = {
        "cruise": "巡航",
        "spin_seek": "原地寻缝",
        "reverse": "倒车",
        "gap_crawl": "穿缝缓行",
        "stop": "急停",
    }
    return names.get(decision, decision)


def _render_sectors(counts, decision: str) -> str:
    """一行 ASCII 扇区图：`-` 空、`+` 有点、`#` 密集，车头在最左。"""
    if counts is None or len(counts) == 0:
        return ""
    maxc = max(1, int(counts.max()))
    chars = []
    for c in counts:
        if c == 0:
            chars.append("·")
        elif c < maxc * 0.5:
            chars.append("+")
        else:
            chars.append("#")
    return f"[{''.join(chars)}] {_fmt_decision(decision)}"


def main() -> int:
    args = build_arg_parser().parse_args()

    # ---- 底盘（可关）-------------------------------------------------
    robot = None
    if not args.no_obstacle:
        from socketcan_chassis import (
            SocketcanBunkerController,
            detect_gs_usb_channel,
        )
        channel = args.can_channel or detect_gs_usb_channel()
        if not channel:
            print(
                "未找到 USB CAN 网卡。请先:\n"
                "  1) 插入 candleLight\n"
                "  2) bash bringup_gs_usb_can.sh\n"
                "  3) ip -br link | grep can"
            )
            return 1
        robot = SocketcanBunkerController(interface="socketcan", channel=channel)
        try:
            robot.start()
        except Exception as exc:
            print(f"打开 CAN 失败: {exc}")
            return 1
        print(f"  底盘控制     : socketcan {channel} (kernel gs_usb)")
        if not robot.enable_and_handshake(timeout_s=5.0):
            # 通道写错（重启后 USB-CAN 接口名可能从 can1 变成 can0 等）是
            # 最常见原因：帮用户列出所有 can 网卡及归属，避免盲试。
            auto = detect_gs_usb_channel()
            print("使能失败。可加 --no-obstacle 仅看点云。")
            if auto and auto != channel:
                print(
                    f"[底盘诊断] ⚠ 当前用的通道是 {channel}，但检测到 USB-CAN 实际在 {auto}。\n"
                    f"  请改用: python3 run_obstacle_avoidance.py --can-channel {auto}\n"
                    f"  （接口名在重启后可能变化，不要硬编码 can1/can0）"
                )
            robot.stop()
            return 2

    # ---- 雷达 ---------------------------------------------------------
    print(f"[Airy] 打开雷达 UDP :{args.msop_port} …")
    lidar = RoboSenseAiry(msop_port=args.msop_port, verbose=True)
    oa = ObstacleAvoidance(
        n_sectors=args.n_sectors,
        front_block_thresh=args.front_thresh,
        side_block_thresh=args.side_thresh,
    )
    if args.cruise != CRUISE_VX:
        DECISION_CMDS["cruise"] = (args.cruise, DECISION_CMDS["cruise"][1])

    print("开始避障巡航（Ctrl+C 停止）…")
    last_seq = 0
    frame_count = 0
    try:
        while True:
            pcd, _intensity, seq = lidar.poll_frame(last_seq, remove_nan=True)
            if pcd is None:
                time.sleep(0.02)
                continue
            last_seq = seq
            frame_count += 1

            counts = points_to_sector_counts(
                pcd, n_sectors=args.n_sectors, max_range_m=args.range)
            decision, v, w = oa.decide(counts)

            if robot is not None:
                if decision == "stop":
                    robot.stop_motion()
                else:
                    robot.set_velocity(v, w)

            if not args.no_viz and frame_count % 10 == 0:
                print(_render_sectors(counts, decision))
            if robot is not None:
                od = robot.latest_odometer
                odo_str = (f"里程 L/R={od.left_wheel_mm}/{od.right_wheel_mm}mm"
                           if od else "里程=—")
                print(f"#{frame_count} {_fmt_decision(decision)} "
                      f"v={v:.2f} w={w:.2f} {odo_str}", flush=True)
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("\n停止 …")
        return 0
    finally:
        if robot is not None:
            robot.stop_motion()
            robot.stop()
        lidar.stop()


if __name__ == "__main__":
    raise SystemExit(main())
