#!/usr/bin/env python3
"""机械臂「抓取完成」信号模拟器 —— 联调/自测用（不在真车上使用）。

机械臂方还没就绪时，用本脚本模拟「机械臂完成抓取」向底盘发信号，验证
任务链的「等待抓取 → 收到信号 → 返回起点」闭环：

用法（三种信号通道，任选其一）::

    # 1) 联调文件信号（无硬件）：向文件写完成标记
    python3 arm_bridge/simulate_arm.py --file /tmp/grasp.sig

    # 2) CAN 报文信号（机械臂方协议一致时）：发 0x3A1 帧，data[0]=0x01
    python3 arm_bridge/simulate_arm.py --can can1
    python3 arm_bridge/simulate_arm.py --can can1 --frame-id 0x3A1

    # 3) GPIO 电平信号（真机接线联调）：拉高指定引脚
    python3 arm_bridge/simulate_arm.py --gpio 18

    # 4) 云端指令（模拟机械臂控制器向云端发 grasp_done）：
    #    在 mock_cloud 控制台直接输入 `grasp_done` / `gd` 即可，无需本脚本。

提示：文件信号在真实现场也可用——机械臂控制器抓取完成后用
``echo grasped > /tmp/grasp.sig`` 即触发，无需改底盘代码。
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

# 脚本位于 arm_bridge/ 子目录，需把父目录（项目根）加入 sys.path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from _bootstrap import ensure_project_root  # noqa: E402

ensure_project_root()


def _send_file(path: str) -> None:
    Path(path).write_text("grasped\n", encoding="utf-8")
    print(f"[模拟机械臂] 已向 {path} 写入完成标记 → 底盘将收到抓取完成")


def _send_can(channel: str, frame_id: int) -> None:
    try:
        import can
    except ImportError:
        print("错误: 需要 python-can (pip install python-can)", file=sys.stderr)
        raise SystemExit(2)
    bus = can.Bus(channel=channel, interface="socketcan")
    msg = can.Message(arbitration_id=frame_id, data=bytes([0x01] + [0] * 7))
    bus.send(msg)
    bus.shutdown()
    print(f"[模拟机械臂] 已在 {channel} 发送帧 0x{frame_id:03X} (data[0]=0x01)")


def _send_gpio(pin: int) -> None:
    try:
        import Jetson.GPIO as gpio
    except ImportError:
        print("错误: 需要 Jetson.GPIO (pip install Jetson.GPIO)", file=sys.stderr)
        raise SystemExit(2)
    gpio.setmode(gpio.BOARD)
    gpio.setup(pin, gpio.OUT)
    gpio.output(pin, gpio.HIGH)
    print(f"[模拟机械臂] 已拉高 BOARD pin {pin} → 底盘将收到抓取完成")
    time.sleep(2.0)  # 保持高电平足够长以便消抖确认
    gpio.cleanup(pin)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="模拟机械臂向底盘发送「抓取完成」信号（联调用）")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--file", metavar="PATH",
                       help="向文件写入完成标记（无硬件联调）")
    group.add_argument("--can", metavar="CHANNEL",
                       help="在 CAN 通道发送抓取完成帧（如 can1）")
    group.add_argument("--gpio", type=int, metavar="PIN",
                       help="拉高 Jetson GPIO 引脚（BOARD 编号）")
    parser.add_argument("--frame-id", type=lambda s: int(s, 0), default=0x3A1,
                        help="CAN 抓取完成帧 ID（默认 0x3A1）")
    args = parser.parse_args()

    try:
        if args.file:
            _send_file(args.file)
        elif args.can:
            _send_can(args.can, args.frame_id)
        elif args.gpio is not None:
            _send_gpio(args.gpio)
    except Exception as exc:
        print(f"[模拟机械臂] 发送失败: {exc}", file=sys.stderr)
        return 1
    print("[模拟机械臂] 完成")
    return 0


if __name__ == "__main__":
    sys.exit(main())
