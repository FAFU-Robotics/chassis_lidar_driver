"""Verify feedback frame parsing."""

import struct

from bunker_mini.protocol import (
    BmsFeedback,
    ControlMode,
    FaultFlags,
    MotionFeedback,
    OdometerFeedback,
    RemoteControlFeedback,
    SystemStatus,
    VehicleState,
)


def test_motion_feedback() -> None:
    fb = MotionFeedback.from_bytes(bytes.fromhex("00960000"))
    assert abs(fb.linear_velocity_m_s - 0.15) < 1e-6
    assert abs(fb.angular_velocity_rad_s - 0.0) < 1e-6


def test_system_status() -> None:
    payload = struct.pack(">BBH", VehicleState.NORMAL, ControlMode.CAN_COMMAND, 240) + b"\x00\x00\x00\x05"
    status = SystemStatus.from_bytes(payload)
    assert status.battery_voltage_v == 24.0
    assert status.control_mode == ControlMode.CAN_COMMAND


def test_odometer() -> None:
    fb = OdometerFeedback.from_bytes(bytes.fromhex("000003E8000007D0"))
    assert fb.left_wheel_mm == 1000
    assert fb.right_wheel_mm == 2000


def test_bms() -> None:
    payload = bytes([80, 95]) + struct.pack(">Hhh", 2400, -50, 250)
    fb = BmsFeedback.from_bytes(payload)
    assert fb.soc_percent == 80
    assert fb.voltage_v == 24.0
    assert fb.current_a == -5.0
    assert fb.temperature_c == 25.0


def test_fault_flags() -> None:
    flags = FaultFlags.from_byte(0b10000101)
    assert flags.battery_undervoltage_fault
    assert flags.emergency_stop
    assert "急停触发" in flags.active_items()
    rc = FaultFlags.from_byte(0x04)
    assert rc.remote_control_lost
    assert rc.active_items(ignore=frozenset({"remote_control_lost"})) == []


def test_remote_swb_command_bit() -> None:
    # bit[2-3]: 2=上(指令) 1=中(遥控)
    cmd = RemoteControlFeedback.from_bytes(bytes([0b00001000, 0, 0, 0, 0, 0, 0, 1]))
    rc = RemoteControlFeedback.from_bytes(bytes([0b00000100, 0, 0, 0, 0, 0, 0, 1]))
    assert cmd.swb_is_command
    assert not rc.swb_is_command


if __name__ == "__main__":
    test_motion_feedback()
    test_system_status()
    test_odometer()
    test_bms()
    test_fault_flags()
    test_remote_swb_command_bit()
    print("feedback parsing OK")
