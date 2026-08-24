#!/usr/bin/env python3
"""Minimal Bunker Mini drive script for Linux socketcan.

Safety:
  - Default speed is low (0.15 m/s).
  - Ctrl+C / timeout always sends zero velocity then disables control.
  - Requires chassis heartbeat (0x211) before moving.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

from _bootstrap import ensure_project_root

ensure_project_root()

from bunker_mini import BunkerMiniController
from bunker_mini.can_util import (
    CanConfigError,
    add_can_cli_args,
    detect_socketcan_channel,
    print_can_config_help,
)
from bunker_mini.protocol import (
    ControlMode,
    FaultFlags,
    VehicleState,
    control_mode_label,
    vehicle_state_label,
)


def wait_heartbeat(ctrl: BunkerMiniController, timeout_s: float) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if ctrl.latest_status is not None:
            return True
        time.sleep(0.05)
    return False


def enable_can_mode(ctrl: BunkerMiniController, timeout_s: float = 2.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        st = ctrl.latest_status
        if st is not None and st.control_mode == ControlMode.REMOTE_CONTROL:
            print(
                "遥控器占用控制权 (REMOTE_CONTROL)。"
                "请拨到「指令/CAN」档后再试。"
            )
            return False
        if st is not None and st.vehicle_state == VehicleState.EMERGENCY_STOP:
            print("检测到急停，尝试清除非关键故障…")
            ctrl.clear_faults()
            time.sleep(0.1)
        ctrl.enable_can_control()
        time.sleep(0.1)
        st = ctrl.latest_status
        if st is not None and st.control_mode == ControlMode.CAN_COMMAND:
            return True
    return False


def print_status(ctrl: BunkerMiniController, prefix: str = "") -> None:
    st = ctrl.latest_status
    mot = ctrl.latest_motion
    if st is None:
        print(f"{prefix}无状态帧")
        return
    faults = FaultFlags.from_byte(st.fault_code).active_items()
    fault_s = ", ".join(faults) if faults else "无"
    mot_s = "无运动反馈"
    if mot is not None:
        mot_s = (
            f"v={mot.linear_velocity_m_s:+.3f} m/s, "
            f"w={mot.angular_velocity_rad_s:+.3f} rad/s"
        )
    print(
        f"{prefix}"
        f"vehicle={vehicle_state_label(st.vehicle_state)} | "
        f"mode={control_mode_label(st.control_mode)} | "
        f"batt={st.battery_voltage_v:.1f}V | hb={st.count} | "
        f"fault={fault_s} | {mot_s}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Drive Bunker Mini over CAN")
    add_can_cli_args(parser)
    parser.add_argument(
        "--action",
        choices=("diagnose", "forward", "back", "left", "right", "stop", "demo"),
        default="diagnose",
        help="动作: diagnose 只读状态; demo 前进→停→左转→停",
    )
    parser.add_argument("--linear", type=float, default=0.15, help="线速度 m/s")
    parser.add_argument("--angular", type=float, default=0.3, help="角速度 rad/s")
    parser.add_argument("--duration", type=float, default=2.0, help="动作持续时间 s")
    parser.add_argument("--hb-timeout", type=float, default=3.0, help="等待心跳超时 s")
    args = parser.parse_args()

    if args.list_devices:
        print_can_config_help()
        return 0

    # 通道留空时自动检测 USB-CAN（接口名重启后会变，勿硬编码）
    channel = args.channel or detect_socketcan_channel() or "can0"
    ctrl: BunkerMiniController | None = None

    def _shutdown(*_args) -> None:
        if ctrl is not None:
            try:
                ctrl.set_velocity(0.0, 0.0)
                time.sleep(0.05)
                ctrl.stop()
            except Exception:
                pass
        print("\n已急停并关闭 CAN。")
        sys.exit(130)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        ctrl = BunkerMiniController(interface=args.interface, channel=channel)
    except CanConfigError as exc:
        print(exc)
        return 1

    ctrl.start()
    print(f"已打开 CAN: interface={args.interface} channel={channel}")
    print("等待底盘心跳 (0x211)…")

    if not wait_heartbeat(ctrl, args.hb_timeout):
        print(
            "未收到底盘状态帧。\n"
            "请检查:\n"
            "  1) 底盘已上电\n"
            "  2) CAN_H/CAN_L/GND 已接到工控机 CAN 网卡（经收发器）\n"
            "  3) 波特率 500000，总线有终端电阻\n"
            f"  4) 执行: sudo ip link set {channel} up type can bitrate 500000\n"
            f"  5) 另开终端: candump {channel}  应能看到 211#..."
        )
        ctrl.stop()
        return 2

    print_status(ctrl, prefix="心跳 OK | ")

    if args.action == "diagnose":
        print("诊断模式：持续打印状态 5 秒（不发运动指令）…")
        t0 = time.time()
        while time.time() - t0 < 5.0:
            print_status(ctrl)
            time.sleep(0.5)
        ctrl.stop()
        return 0

    if not enable_can_mode(ctrl):
        print("无法进入 CAN 指令模式。")
        print_status(ctrl, prefix="当前 | ")
        ctrl.stop()
        return 3

    print_status(ctrl, prefix="已使能 | ")

    def run_motion(linear: float, angular: float, duration: float, label: str) -> None:
        print(f"→ {label}: v={linear:+.2f} m/s, w={angular:+.2f} rad/s, {duration:.1f}s")
        ctrl.set_velocity(linear, angular)
        t0 = time.time()
        while time.time() - t0 < duration:
            print_status(ctrl, prefix="  ")
            time.sleep(0.2)
        ctrl.set_velocity(0.0, 0.0)
        time.sleep(0.3)
        print_status(ctrl, prefix="  停 | ")

    v = abs(args.linear)
    w = abs(args.angular)

    try:
        if args.action == "forward":
            run_motion(+v, 0.0, args.duration, "前进")
        elif args.action == "back":
            run_motion(-v, 0.0, args.duration, "后退")
        elif args.action == "left":
            run_motion(0.0, +w, args.duration, "左转")
        elif args.action == "right":
            run_motion(0.0, -w, args.duration, "右转")
        elif args.action == "stop":
            run_motion(0.0, 0.0, 0.5, "停车")
        elif args.action == "demo":
            run_motion(+v, 0.0, args.duration, "前进")
            run_motion(0.0, 0.0, 0.5, "停")
            run_motion(0.0, +w, args.duration, "左转")
            run_motion(0.0, 0.0, 0.5, "停")
    finally:
        ctrl.set_velocity(0.0, 0.0)
        time.sleep(0.05)
        ctrl.stop()
        print("完成，已停车并关闭总线。")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
