#!/usr/bin/env python3
"""Example: enable CAN mode and drive BUNKER MINI 2.0 for a short sequence."""

from __future__ import annotations

import argparse
import sys
import time

from _bootstrap import ensure_project_root

ensure_project_root()

from bunker_mini import BunkerMiniController, ControlMode, VehicleState
from bunker_mini.can_util import CanConfigError, add_can_cli_args, format_device_list, print_can_config_help


def main() -> int:
    parser = argparse.ArgumentParser(description="BUNKER MINI 2.0 CAN control demo")
    add_can_cli_args(parser)
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

    def print_status(status) -> None:
        print(
            f"[status] vehicle={status.vehicle_state.name} "
            f"mode={status.control_mode.name} "
            f"battery={status.battery_voltage_v:.1f}V fault=0x{status.fault_code:02X}"
        )

    def print_motion(motion) -> None:
        print(
            f"[motion] linear={motion.linear_velocity_m_s:.3f} m/s "
            f"angular={motion.angular_velocity_rad_s:.3f} rad/s"
        )

    controller.on_status(print_status)
    controller.on_motion_feedback(print_motion)

    try:
        controller.start()
        print("Waiting for chassis status...")
        for _ in range(50):
            status = controller.latest_status
            if status is not None:
                break
            time.sleep(0.05)
        else:
            print("No status frame received on 0x211. Check CAN wiring and bitrate (500K).")
            return 1

        if status.vehicle_state != VehicleState.NORMAL:
            print(f"Vehicle not ready: {status.vehicle_state.name}")
            return 1

        if status.control_mode == ControlMode.REMOTE_CONTROL:
            print("Remote control is active and has priority. Switch RC to command mode first.")
            return 1

        print("Enabling CAN command mode (0x421 -> 0x01)...")
        controller.enable_can_control()
        time.sleep(0.2)

        print("Forward 0.15 m/s for 2 s (doc example: 0x111 payload 00 96 00 00 ...)")
        controller.set_velocity(0.15, 0.0)
        time.sleep(2.0)

        print("Rotate 0.2 rad/s for 2 s (doc example: 0x111 payload 00 00 00 C8 ...)")
        controller.set_velocity(0.0, 0.2)
        time.sleep(2.0)

        print("Stopping...")
        controller.stop_motion()
        time.sleep(0.5)
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted")
        return 130
    finally:
        controller.stop()


if __name__ == "__main__":
    sys.exit(main())
