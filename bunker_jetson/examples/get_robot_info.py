#!/usr/bin/env python3
"""Example: read current BUNKER MINI 2.0 chassis information over CAN."""

from __future__ import annotations

import argparse
import logging
import sys
import time
import warnings

# ── suppress python-can import noise ──
logging.basicConfig(level=logging.WARNING, format="%(message)s")
logging.getLogger("can").setLevel(logging.ERROR)
warnings.filterwarnings("ignore")

from _bootstrap import ensure_project_root

ensure_project_root()

from bunker_mini.can_util import CanConfigError, add_can_cli_args, format_device_list, print_can_config_help, quiet_can_detection
from bunker_mini.monitor import BunkerMiniMonitor, RobotSnapshot
from bunker_mini.protocol import FaultFlags, control_mode_label, vehicle_state_label


def _no_traffic_help() -> list[str]:
    return [
        "未收到任何 CAN 数据 — 脚本本身正常，但底盘没有向总线发送报文。",
        "",
        "get_robot_info.py 的作用:",
        "  通过 CAN 总线读取 BUNKER MINI 2.0 底盘上报的状态（只读，不控制运动）。",
        "  必须物理连接 CAN 且底盘上电，才会显示电池、速度、里程等信息。",
        "",
        "请逐项检查:",
        "  1. 小车是否已开机（底盘指示灯/风扇有反应）",
        "  2. USB-CAN 适配器 CAN_H / CAN_L 是否接到小车 CAN 口",
        "  3. CAN 设备是否正确识别（--list-devices 查看，蓝牙 COM 口不会有 CAN 数据）",
        "  4. 如果使用 candleLight: 确认已用 Zadig 安装 WinUSB 驱动",
        "  5. 波特率是否为 500K（BUNKER MINI 2.0 固定要求）",
        "  6. 适配器是否已被其他软件占用（关闭 CANalyst、ZCANPRO 等）",
        "",
        "建议尝试:",
        "  python examples/get_robot_info.py --list-devices",
        "  # candleLight:",
        "  python examples/get_robot_info.py --interface gs_usb",
        "  # 串口 CAN:",
        "  python examples/get_robot_info.py --interface slcan --channel COMx",
    ]


def format_snapshot(snapshot: RobotSnapshot) -> str:
    lines: list[str] = []
    lines.append("=" * 48)
    lines.append(f"采集时间: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(snapshot.collected_at))}")
    lines.append(f"CAN 总帧数: {snapshot.total_frames}")
    lines.append(
        f"已识别帧: {', '.join(f'0x{i:03X}' for i in sorted(snapshot.received_ids)) or '无'}"
    )

    if snapshot.has_bus_traffic and not snapshot.is_connected:
        unknown = sorted(snapshot.all_frame_ids - snapshot.received_ids)
        if unknown:
            lines.append(
                f"其他 CAN ID: {', '.join(f'0x{i:03X}' for i in unknown)}"
            )

    if not snapshot.is_connected:
        lines.append("")
        if not snapshot.has_bus_traffic:
            lines.extend(_no_traffic_help())
        else:
            lines.append("CAN 总线上有数据，但未收到 BUNKER MINI 系统状态帧 (0x211)。")
            lines.append("可能原因: 波特率不匹配、接错设备、或协议版本不同。")
        lines.append("=" * 48)
        return "\n".join(lines)

    status = snapshot.system_status
    assert status is not None
    faults = FaultFlags.from_byte(status.fault_code)

    lines.append("")
    lines.append("[系统状态 0x211]")
    lines.append(f"  车体状态: {vehicle_state_label(status.vehicle_state)}")
    lines.append(f"  控制模式: {control_mode_label(status.control_mode)}")
    lines.append(f"  电池电压: {status.battery_voltage_v:.1f} V")
    lines.append(f"  故障码:   0x{status.fault_code:02X}")
    if faults.active_items():
        lines.append(f"  故障详情: {', '.join(faults.active_items())}")
    else:
        lines.append("  故障详情: 无")
    lines.append(f"  计数校验: {status.count}")

    if snapshot.motion is not None:
        motion = snapshot.motion
        lines.append("")
        lines.append("[运动反馈 0x221]")
        lines.append(f"  线速度: {motion.linear_velocity_m_s:.3f} m/s")
        lines.append(f"  角速度: {motion.angular_velocity_rad_s:.3f} rad/s")

    if snapshot.odometer is not None:
        odom = snapshot.odometer
        lines.append("")
        lines.append("[里程计 0x311]")
        lines.append(f"  左轮累计: {odom.left_wheel_mm} mm ({odom.left_wheel_mm / 1000:.3f} m)")
        lines.append(f"  右轮累计: {odom.right_wheel_mm} mm ({odom.right_wheel_mm / 1000:.3f} m)")

    if snapshot.bms is not None:
        bms = snapshot.bms
        lines.append("")
        lines.append("[BMS 0x361]")
        lines.append(f"  SOC: {bms.soc_percent} %")
        lines.append(f"  SOH: {bms.soh_percent} %")
        lines.append(f"  电压: {bms.voltage_v:.2f} V")
        lines.append(f"  电流: {bms.current_a:.1f} A")
        lines.append(f"  温度: {bms.temperature_c:.1f} °C")

    if snapshot.remote is not None:
        rc = snapshot.remote
        lines.append("")
        lines.append("[遥控器 0x241]")
        lines.append(f"  开关状态: 0x{rc.switch_bits:02X}")
        lines.append(f"  右摇杆: LR={rc.right_stick_lr:+4d}  UD={rc.right_stick_ud:+4d}")
        lines.append(f"  左摇杆: LR={rc.left_stick_lr:+4d}  UD={rc.left_stick_ud:+4d}")
        lines.append(f"  VRA 旋钮: {rc.knob_vra:+4d}")

    lines.append("=" * 48)
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description="Read BUNKER MINI 2.0 chassis information")
    add_can_cli_args(parser)
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Seconds to listen for feedback frames (default: 5.0)",
    )
    parser.add_argument(
        "--watch",
        action="store_true",
        help="Continuously print updates until Ctrl+C",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Refresh interval in watch mode (default: 1.0 s)",
    )
    args = parser.parse_args()

    if args.list_devices:
        with quiet_can_detection():
            print(format_device_list())
        return 0

    try:
        monitor = BunkerMiniMonitor(channel=args.channel, interface=args.interface)
    except CanConfigError as exc:
        print(f"CAN 配置错误:\n{exc}", file=sys.stderr)
        print(file=sys.stderr)
        print_can_config_help()
        return 2

    print(f"CAN 接口: {args.interface}, 通道: {args.channel or '(自动检测)'}")

    try:
        if args.watch:
            print("监听中，按 Ctrl+C 退出...\n")
            while True:
                snapshot = monitor.collect(duration_s=args.interval)
                print(format_snapshot(snapshot))
                print()
        else:
            print(f"正在采集 {args.duration:.1f} s 内的底盘反馈...\n")
            snapshot = monitor.collect(duration_s=args.duration)
            print(format_snapshot(snapshot))
            return 0 if snapshot.is_connected else 1
    except KeyboardInterrupt:
        print("\n已退出")
        return 130
    finally:
        monitor.close()


if __name__ == "__main__":
    sys.exit(main())
