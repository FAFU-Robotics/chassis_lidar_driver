"""BUNKER MINI 2.0 CAN protocol definitions (section 3.3.2).

Communication: CAN 2.0B, 500 kbps, Motorola (big-endian) byte order.
手册写 20 ms / 50 Hz；遥控按 10 ms / 100 Hz 下发（底盘 500 ms 无帧才停，
更快合法）。导航等未改周期的环仍可按 20 ms 跑。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Final

CAN_BITRATE: Final = 500_000
CONTROL_PERIOD_S: Final = 0.01
CONTROL_TIMEOUT_S: Final = 0.5


class CanId(IntEnum):
    MOTION_CONTROL = 0x111
    SYSTEM_STATUS = 0x211
    MOTION_FEEDBACK = 0x221
    REMOTE_CONTROL = 0x241
    MOTOR_HIGH_SPEED_BASE = 0x252  # 0x252~0x253
    MOTOR_LOW_SPEED_BASE = 0x262  # 0x262~0x263
    ODOMETER = 0x311
    BMS = 0x361
    BMS_ALARM = 0x362
    MODE_SETTING = 0x421
    FAULT_CLEAR = 0x441


class VehicleState(IntEnum):
    NORMAL = 0x00
    EMERGENCY_STOP = 0x01
    SYSTEM_EXCEPTION = 0x02


class ControlMode(IntEnum):
    STANDBY = 0x00
    CAN_COMMAND = 0x01
    REMOTE_CONTROL = 0x03


class FaultClearCommand(IntEnum):
    CLEAR_NON_CRITICAL = 0x00
    CLEAR_MOTOR_2 = 0x01  # 手册 Table 3.6: 清除 2 号电机错误
    CLEAR_MOTOR_3 = 0x02  # 手册 Table 3.6: 清除 3 号电机错误


@dataclass(frozen=True)
class MotionCommand:
    """Motion control frame (ID 0x111)."""

    linear_velocity_mm_s: int
    angular_velocity_mrad_s: int

    def __post_init__(self) -> None:
        if not -1300 <= self.linear_velocity_mm_s <= 1300:
            raise ValueError("linear velocity must be in [-1300, 1300] mm/s")
        if not -2000 <= self.angular_velocity_mrad_s <= 2000:
            raise ValueError("angular velocity must be in [-2000, 2000] (0.001 rad/s)")

    @classmethod
    def from_si(cls, linear_m_s: float, angular_rad_s: float) -> MotionCommand:
        return cls(
            linear_velocity_mm_s=int(round(linear_m_s * 1000.0)),
            angular_velocity_mrad_s=int(round(angular_rad_s * 1000.0)),
        )

    def to_bytes(self) -> bytes:
        payload = struct.pack(
            ">hh",
            self.linear_velocity_mm_s,
            self.angular_velocity_mrad_s,
        )
        return payload + b"\x00" * 4


@dataclass(frozen=True)
class SystemStatus:
    vehicle_state: VehicleState
    control_mode: ControlMode
    battery_voltage_v: float
    fault_code: int
    count: int

    @classmethod
    def from_bytes(cls, data: bytes) -> SystemStatus:
        if len(data) < 8:
            raise ValueError("system status frame must be 8 bytes")
        vehicle_state, control_mode, voltage_raw = struct.unpack(">BBH", data[:4])
        fault_code = data[5]
        count = data[7]
        return cls(
            vehicle_state=VehicleState(vehicle_state),
            control_mode=ControlMode(control_mode),
            battery_voltage_v=voltage_raw / 10.0,
            fault_code=fault_code,
            count=count,
        )


@dataclass(frozen=True)
class MotionFeedback:
    """Motion feedback frame (ID 0x221).

    Scaling (calibrated against real chassis behavior):
      byte[0:1] linear speed ×1000 → m/s   (0.001 m/s per LSB)
      byte[2:3] angular velocity ×1000 → rad/s (0.001 rad/s per LSB)

    注意：早期版本把角速度除以 100（按 0.01 rad/s 解析），导致轨迹录制里
    存的角速度比实际大 10 倍，回放时小车狂转。修正为 ×1000，与指令侧
    0x111（0.001 rad/s 每 LSB）和线速度字段保持一致。
    """

    linear_velocity_m_s: float
    angular_velocity_rad_s: float

    @classmethod
    def from_bytes(cls, data: bytes) -> MotionFeedback:
        if len(data) < 4:
            raise ValueError("motion feedback frame must contain at least 4 bytes")
        linear_raw, angular_raw = struct.unpack(">hh", data[:4])
        return cls(
            linear_velocity_m_s=linear_raw / 1000.0,
            angular_velocity_rad_s=angular_raw / 1000.0,
        )


def encode_mode_setting(enable_can_control: bool) -> bytes:
    """Control mode setting frame (ID 0x421)."""
    return bytes([ControlMode.CAN_COMMAND if enable_can_control else ControlMode.STANDBY])


def encode_fault_clear(command: FaultClearCommand) -> bytes:
    """State setting / fault clear frame (ID 0x441)."""
    return bytes([command])


@dataclass(frozen=True)
class FaultFlags:
    battery_undervoltage_fault: bool
    battery_undervoltage_warning: bool
    remote_control_lost: bool
    drive2_comm_fault: bool
    drive3_comm_fault: bool
    emergency_stop: bool

    @classmethod
    def from_byte(cls, value: int) -> FaultFlags:
        return cls(
            battery_undervoltage_fault=bool(value & (1 << 0)),
            battery_undervoltage_warning=bool(value & (1 << 1)),
            remote_control_lost=bool(value & (1 << 2)),
            drive2_comm_fault=bool(value & (1 << 4)),
            drive3_comm_fault=bool(value & (1 << 5)),
            emergency_stop=bool(value & (1 << 7)),
        )

    def active_items(self, *, ignore: frozenset | None = None) -> list[str]:
        labels = {
            "battery_undervoltage_fault": "电池欠压故障(10%)",
            "battery_undervoltage_warning": "电池欠压警告(15%)",
            "remote_control_lost": "遥控器失联保护",
            "drive2_comm_fault": "驱动2通讯故障",
            "drive3_comm_fault": "驱动3通讯故障",
            "emergency_stop": "急停触发",
        }
        skip = ignore or frozenset()
        return [
            labels[name]
            for name, active in self.__dict__.items()
            if active and name not in skip
        ]


@dataclass(frozen=True)
class OdometerFeedback:
    left_wheel_mm: int
    right_wheel_mm: int

    @classmethod
    def from_bytes(cls, data: bytes) -> OdometerFeedback:
        if len(data) < 8:
            raise ValueError("odometer frame must be 8 bytes")
        left, right = struct.unpack(">ii", data[:8])
        return cls(left_wheel_mm=left, right_wheel_mm=right)


@dataclass(frozen=True)
class BmsFeedback:
    soc_percent: int
    soh_percent: int
    voltage_v: float
    current_a: float
    temperature_c: float

    @classmethod
    def from_bytes(cls, data: bytes) -> BmsFeedback:
        if len(data) < 8:
            raise ValueError("BMS frame must be 8 bytes")
        soc, soh, voltage_raw, current_raw, temp_raw = struct.unpack(">BBHhh", data[:8])
        return cls(
            soc_percent=soc,
            soh_percent=soh,
            voltage_v=voltage_raw / 100.0,
            current_a=current_raw / 10.0,
            temperature_c=temp_raw / 10.0,
        )


@dataclass(frozen=True)
class RemoteControlFeedback:
    switch_bits: int
    right_stick_lr: int
    right_stick_ud: int
    left_stick_ud: int
    left_stick_lr: int
    knob_vra: int
    count: int

    @classmethod
    def from_bytes(cls, data: bytes) -> RemoteControlFeedback:
        if len(data) < 7:
            raise ValueError(f"remote control frame too short: {len(data)} bytes, need >= 7")
        # Prevent struct.unpack crash when frame is 7 bytes (missing trailing count).
        # Pad to 8 bytes with a zero byte so format >bbbbbbxB still parses.
        padded = data[:7].ljust(8, b"\x00")
        sw, r_lr, r_ud, l_ud, l_lr, vra, count = struct.unpack(">bbbbbbxB", padded)
        return cls(
            switch_bits=sw & 0xFF,
            right_stick_lr=r_lr,
            right_stick_ud=r_ud,
            left_stick_ud=l_ud,
            left_stick_lr=l_lr,
            knob_vra=vra,
            count=count,
        )

    @property
    def swb_is_command(self) -> bool:
        """FS 遥控 SWB：bit[2-3]，2=上(指令档) 1=中(遥控档) 3=下。"""
        return ((self.switch_bits >> 2) & 0x3) == 2


_VEHICLE_STATE_LABELS = {
    VehicleState.NORMAL: "正常",
    VehicleState.EMERGENCY_STOP: "急停",
    VehicleState.SYSTEM_EXCEPTION: "系统异常",
}

_CONTROL_MODE_LABELS = {
    ControlMode.STANDBY: "待机",
    ControlMode.CAN_COMMAND: "CAN 指令模式",
    ControlMode.REMOTE_CONTROL: "遥控模式",
}


def vehicle_state_label(state: VehicleState) -> str:
    return _VEHICLE_STATE_LABELS.get(state, state.name)


def control_mode_label(mode: ControlMode) -> str:
    return _CONTROL_MODE_LABELS.get(mode, mode.name)
