"""CAN configuration helper tests."""

import types

import pytest

from bunker_mini.can_util import (
    CanConfigError,
    candidate_socketcan_channels,
    default_interface,
    detect_socketcan_channel,
    probe_chassis_channel,
    recover_unhealthy_socketcan,
    resolve_can_config,
)


def test_linux_defaults(monkeypatch) -> None:
    # Linux 平台默认使用 socketcan（与是否插有 USB 设备无关）
    monkeypatch.setattr("bunker_mini.can_util.is_windows", lambda: False)
    assert default_interface() == "socketcan"


def test_windows_defaults_without_usb(monkeypatch) -> None:
    # Windows 上若未检测到 candleLight 设备，默认回退 slcan
    monkeypatch.setattr("bunker_mini.can_util.is_windows", lambda: True)
    monkeypatch.setattr("bunker_mini.can_util._detect_usb_can_devices", lambda: [])
    assert default_interface() == "slcan"


def test_resolve_socketcan_linux(monkeypatch) -> None:
    monkeypatch.setattr("bunker_mini.can_util.is_windows", lambda: False)
    # 固定环境：只有一个 can0（up），保证测试不依赖宿主真实网卡状态
    _patch_socketcan(monkeypatch, {"can0": "up"}, usb=set())
    ch, iface = resolve_can_config(None, "socketcan", allow_auto_channel=False)
    assert ch == "can0"
    assert iface == "socketcan"


def test_slcan_requires_channel(monkeypatch) -> None:
    monkeypatch.setattr("bunker_mini.can_util.is_windows", lambda: True)
    monkeypatch.setattr("bunker_mini.can_util.list_serial_ports", lambda: [])
    with pytest.raises(CanConfigError, match="--channel"):
        resolve_can_config(None, "slcan", allow_auto_channel=False)


def test_windows_rejects_socketcan(monkeypatch) -> None:
    monkeypatch.setattr("bunker_mini.can_util.is_windows", lambda: True)
    with pytest.raises(CanConfigError, match="socketcan"):
        resolve_can_config("can0", "socketcan", allow_auto_channel=False)


# ----------------------------------------------------------------------
# 通道选择：USB+UP > USB+DOWN > 板载 UP。
# candleLight 掉线时绝不能改绑 Jetson mttcan，否则 0x111 打空总线 bus-off。
# ----------------------------------------------------------------------


def _patch_socketcan(monkeypatch, states: dict[str, str], usb: set[str]) -> None:
    """Patch can_util's /sys/class/net view with a fake set of can ifaces."""
    monkeypatch.setattr(
        "bunker_mini.can_util.list_socketcan_interfaces",
        lambda: sorted(states),
    )
    monkeypatch.setattr(
        "bunker_mini.can_util._socketcan_operstate",
        lambda ch: states.get(ch, "unknown"),
    )
    monkeypatch.setattr(
        "bunker_mini.can_util._is_usb_netdev",
        lambda ch: ch in usb,
    )
    monkeypatch.setattr(
        "bunker_mini.can_util.socketcan_ctrl_state",
        lambda ch: "error-active",
    )
    monkeypatch.setattr(
        "bunker_mini.can_util.prepare_listen_only",
        lambda ch, bitrate=500000: True,
    )
    monkeypatch.setattr(
        "bunker_mini.can_util.bring_socketcan_up",
        lambda ch, bitrate=500000, listen_only=False: True,
    )
    monkeypatch.setattr(
        "bunker_mini.can_util.ensure_all_socketcan_up",
        lambda: None,
    )
    monkeypatch.setattr(
        "bunker_mini.can_util.restore_tx_mode",
        lambda channels=None: None,
    )
    monkeypatch.setattr(
        "bunker_mini.can_util.leave_listen_only",
        lambda ch, bitrate=500000: True,
    )


def test_candidates_rank_up_usb_first(monkeypatch) -> None:
    _patch_socketcan(
        monkeypatch,
        {"can0": "up", "can1": "down", "can2": "up"},
        usb={"can2"},
    )
    # can2(USB+UP) 第一，can1(USB+DOWN) 第二，can0(板载 UP) 第三
    assert candidate_socketcan_channels() == ["can2", "can1", "can0"]


def test_detect_prefers_usb_even_if_down(monkeypatch) -> None:
    """USB-CAN 即使 DOWN 也优先于板载 UP，避免误绑 mttcan。"""
    _patch_socketcan(monkeypatch, {"can0": "up", "can1": "down"}, usb={"can1"})
    assert detect_socketcan_channel() == "can1"


def test_detect_picks_usb_up_first(monkeypatch) -> None:
    """USB+UP 仍优先于板载 UP（底盘接 USB-CAN 的标准场景）。"""
    _patch_socketcan(monkeypatch, {"can0": "up", "can1": "up"}, usb={"can1"})
    assert detect_socketcan_channel() == "can1"


def test_candidates_demote_error_passive(monkeypatch) -> None:
    """ERROR-PASSIVE 不再当健康口排第一（否则探测会锁死口）。"""
    _patch_socketcan(monkeypatch, {"can0": "up", "can1": "up"}, usb={"can0"})
    monkeypatch.setattr(
        "bunker_mini.can_util.socketcan_ctrl_state",
        lambda ch: "error-passive" if ch == "can0" else "error-active",
    )
    ranked = candidate_socketcan_channels()
    assert ranked[0] == "can1"
    assert ranked[-1] == "can0"


def test_recover_resets_error_passive(monkeypatch) -> None:
    _patch_socketcan(monkeypatch, {"can0": "up", "can1": "up"}, usb={"can0"})
    monkeypatch.setattr(
        "bunker_mini.can_util.socketcan_ctrl_state",
        lambda ch: "error-passive" if ch == "can0" else "error-active",
    )
    reset: list[str] = []
    monkeypatch.setattr(
        "bunker_mini.can_util.reset_socketcan_controller",
        lambda ch: reset.append(ch) or True,
    )
    assert recover_unhealthy_socketcan() == ["can0"]
    assert reset == ["can0"]


# ----------------------------------------------------------------------
# probe：启动时找「能收到底盘反馈帧」的通道
# ----------------------------------------------------------------------


class _FakeBus:
    def __init__(self, frames: list[int]) -> None:
        self._frames = frames
        self.shutdown_calls = 0

    def recv(self, timeout=None):
        if self._frames:
            aid = self._frames.pop(0)
            return types.SimpleNamespace(arbitration_id=aid)
        return None

    def shutdown(self) -> None:
        self.shutdown_calls += 1


def test_probe_finds_channel_with_chassis_feedback(monkeypatch) -> None:
    _patch_socketcan(monkeypatch, {"can0": "up", "can1": "up"}, usb={"can1"})
    monkeypatch.setattr(
        "bunker_mini.can_util.create_can_bus",
        lambda ch, iface: _FakeBus([0x211] if ch == "can1" else []),
    )
    monkeypatch.setattr(
        "bunker_mini.can_util.ensure_socketcan_interface", lambda ch: None)
    assert probe_chassis_channel(timeout_s=0.5) == "can1"


def test_probe_returns_none_when_no_feedback(monkeypatch) -> None:
    _patch_socketcan(monkeypatch, {"can0": "up"}, usb=set())
    monkeypatch.setattr(
        "bunker_mini.can_util.create_can_bus",
        lambda ch, iface: _FakeBus([]),
    )
    monkeypatch.setattr(
        "bunker_mini.can_util.ensure_socketcan_interface", lambda ch: None)
    assert probe_chassis_channel(timeout_s=0.2) is None


def test_probe_hot_reprobe_catches_late_channel(monkeypatch) -> None:
    """重探场景：启动时只有 can0，底盘稍后出现在新上线的 can1 上。
    probe 应能发现新通道（即使候选顺序里 can0 在前）。"""
    _patch_socketcan(monkeypatch, {"can0": "up", "can1": "up"}, usb={"can1"})
    monkeypatch.setattr(
        "bunker_mini.can_util.create_can_bus",
        lambda ch, iface: _FakeBus([0x211, 0x221] if ch == "can1" else []),
    )
    monkeypatch.setattr(
        "bunker_mini.can_util.ensure_socketcan_interface", lambda ch: None)
    # 并行听：只有 can1 有 0x211 → 选定 can1
    assert probe_chassis_channel(timeout_s=0.5) == "can1"


def test_probe_prefers_usb_when_both_hear(monkeypatch) -> None:
    """两口同时听到 0x211 时选 USB-CAN（不按 can0/can1 名字）。"""
    _patch_socketcan(monkeypatch, {"can0": "up", "can1": "up"}, usb={"can1"})
    monkeypatch.setattr(
        "bunker_mini.can_util.create_can_bus",
        lambda ch, iface: _FakeBus([0x211]),
    )
    monkeypatch.setattr(
        "bunker_mini.can_util.ensure_socketcan_interface", lambda ch: None)
    monkeypatch.setattr(
        "bunker_mini.can_util.ensure_all_socketcan_up", lambda: None)
    assert probe_chassis_channel(timeout_s=0.5) == "can1"

