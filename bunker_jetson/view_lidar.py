#!/usr/bin/env python3
"""Live view of the RoboSense Airy LiDAR — verify the UDP link & obstacle map.

在连接小车 CAN 之前，先单独验证雷达链路：本脚本只收雷达数据，不驱动底盘。

Requirements
    电脑配静态 IP 192.168.1.102（雷达出厂 192.168.1.200），雷达上电。

Usage
    # 默认收 0.0.0.0:6699 的 MSOP 包，实时打印 360° 障碍扇区图
    python examples\view_lidar.py

    # 雷达 0° 相对车头有偏置时指定（左转为正），校准 mount_yaw 用
    python examples\view_lidar.py --mount-yaw 90

    # 打印完整点云统计（每帧点数/最近点）而不是障碍图
    python examples\view_lidar.py --points
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

from _bootstrap import ensure_project_root

ensure_project_root()

from bunker_mini.lidar import AiryLidar, LidarError, MSOP_PORT, ObstacleSectors, SelfMaskConfig


def _render_polar_map(sectors: ObstacleSectors) -> list[str]:
    """Render the obstacle map as a simple ASCII polar diagram.

    车头（0°）在顶部中间；半径方向表示距离（0~1.5 m 内按比例缩放），
    '*' 表示该扇区有障碍。
    """
    ROWS, COLS = 17, 65
    grid: list[list[str]] = [[" "] * COLS for _ in range(ROWS)]
    cx, cy = COLS // 2, ROWS - 2
    scale = (ROWS - 3) / 1.5  # 1.5 m → 图高
    for deg in range(0, 360):
        d = sectors.nearest_in_range(deg, 2)
        if d is None or d > 1.5:
            continue
        r = max(1, int(d * scale))
        theta = (deg + 90.0) * math.pi / 180.0  # 0° 在顶部
        x = int(cx + r * math.cos(theta))
        y = int(cy - r * math.sin(theta))
        if 0 <= x < COLS and 0 <= y < ROWS:
            grid[y][x] = "*"

    lines = ["     " + "0" + " " * 63]
    lines.append(" 前(0°)  " + "-" * (COLS - 7))
    for i in range(ROWS):
        lines.append("        " + "".join(grid[i]).rstrip())
    lines.append(" 距离刻度: '*' = 障碍, 每行约 0.1 m")
    return lines


def main() -> int:
    parser = argparse.ArgumentParser(description="Live view of RoboSense Airy LiDAR")
    parser.add_argument("--host", default="0.0.0.0", help="UDP 绑定地址 (默认 0.0.0.0)")
    parser.add_argument("--port", type=int, default=MSOP_PORT, help=f"MSOP UDP 端口 (默认 {MSOP_PORT})")
    parser.add_argument("--mount-yaw", type=float, default=0.0, help="雷达 0° 相对车头偏置（度，左正）")
    parser.add_argument("--pitch", type=float, default=0.0, help="雷达安装俯仰角（度，正=抬头）")
    parser.add_argument("--height", type=float, default=0.0, help="雷达光心离地高度（米），点云 z 平移为离地高度")
    parser.add_argument("--no-self-mask", action="store_true", help="关闭自扫硬件过滤（默认开启）")
    parser.add_argument("--points", action="store_true", help="打印点云统计而非障碍图")
    parser.add_argument("--max-vertical-deg", type=float, default=15.0, help="障碍图最高垂直角 (默认 15)")
    args = parser.parse_args()

    try:
        lidar = AiryLidar(
            host=args.host,
            port=args.port,
            mount_yaw_deg=args.mount_yaw,
            pitch_deg=args.pitch,
            lidar_height_m=args.height,
            self_mask=SelfMaskConfig(enabled=not args.no_self_mask),
            max_vertical_deg=args.max_vertical_deg,
        )
    except LidarError as exc:
        print(f"雷达初始化失败: {exc}", file=sys.stderr)
        return 2

    lidar.start()
    print(f"正在监听 UDP {args.host}:{args.port}（MSOP）... Ctrl+C 退出")
    print("提示: 电脑需配静态 IP 192.168.1.102，雷达默认 192.168.1.200")

    try:
        while True:
            frame = lidar.latest_frame
            if frame is None:
                print(f"\r等待雷达数据... (收到包 {lidar.packet_count}, 无效包 {lidar.bad_packet_count})", end="")
                time.sleep(0.3)
                continue

            ts = time.strftime("%H:%M:%S", time.localtime(frame.received_at))
            print("\033[2J\033[H", end="")  # 清屏
            print(f"=== Airy LiDAR  {ts}  帧 #{lidar.frame_count}  "
                  f"包 {lidar.packet_count}  无效包 {lidar.bad_packet_count}  雷达{'在线' if lidar.is_receiving else '离线'} ===")
            print(f"本帧点数: {len(frame.points)}   最近障碍: "
                  f"{frame.obstacle_sectors.min_distance():.2f} m"
                  if frame.obstacle_sectors.min_distance() is not None
                  else f"本帧点数: {len(frame.points)}")

            if args.points:
                # 简单点云统计：按水平 45° 分区打印最近点
                print("点云最近点(按45°分区, 车头0°):")
                for deg in range(0, 360, 45):
                    d = frame.obstacle_sectors.nearest_in_range(deg, 45)
                    print(f"  {deg:3d}°: {'%.2f m' % d if d is not None else '—'}")
            else:
                print("\n".join(_render_polar_map(frame.obstacle_sectors)))

            print("\n" + frame.obstacle_sectors.as_text(step_deg=15, width=48))
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n已退出")
    finally:
        lidar.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
