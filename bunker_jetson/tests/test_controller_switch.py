"""BunkerMiniController 通道热切换（switch_channel）测试。"""

import time

import pytest

import bunker_mini.controller as controller_mod
from bunker_mini.controller import BunkerMiniController


class _FakeBus:
    def __init__(self, channel: str) -> None:
        self.channel = channel
        self.shutdown_calls = 0
        self.sent: list[int] = []

    def recv(self, timeout=None):
        # 模拟真实 CAN 接收阻塞（避免 RX 线程忙转耗尽 CPU）
        time.sleep(0.02)
        return None

    def send(self, msg) -> None:
        self.sent.append(int(msg.arbitration_id))

    def shutdown(self) -> None:
        self.shutdown_calls += 1


@pytest.fixture
def ctrl(monkeypatch) -> BunkerMiniController:
    buses: list[_FakeBus] = []

    def _fake_create_can_bus(channel, interface, **kwargs):
        bus = _FakeBus(channel)
        buses.append(bus)
        return bus

    monkeypatch.setattr(controller_mod, "create_can_bus", _fake_create_can_bus)
    c = BunkerMiniController(channel="can0", interface="socketcan")
    c.start()
    time.sleep(0.05)  # 让 RX/TX 线程起来
    c._buses = buses
    yield c
    c.stop()


def test_switch_channel_rebuilds_bus(ctrl) -> None:
    old_bus = ctrl._bus
    ctrl.switch_channel("can1")
    time.sleep(0.05)
    assert ctrl._channel == "can1"
    assert ctrl._bus is not old_bus
    assert ctrl._bus.channel == "can1"
    # 旧总线应被 shutdown
    assert old_bus.shutdown_calls >= 1


def test_switch_channel_same_channel_noop(ctrl) -> None:
    old_bus = ctrl._bus
    ctrl.switch_channel("can0")
    time.sleep(0.05)
    assert ctrl._bus is old_bus
    assert ctrl._channel == "can0"


def test_tx_quiet_until_chassis_heard(ctrl) -> None:
    time.sleep(0.08)
    assert 0x111 not in ctrl._bus.sent
    assert 0x421 not in ctrl._bus.sent


def test_tx_sends_when_driving_without_status(ctrl) -> None:
    ctrl.set_velocity_now(0.2, 0.0)
    time.sleep(0.08)
    assert 0x111 in ctrl._bus.sent
    assert 0x421 in ctrl._bus.sent


def test_tx_holds_can_mode_frame(ctrl) -> None:
    from bunker_mini.protocol import ControlMode, SystemStatus, VehicleState

    ctrl._latest_status = SystemStatus(
        vehicle_state=VehicleState.NORMAL,
        control_mode=ControlMode.CAN_COMMAND,
        battery_voltage_v=24.0,
        fault_code=0,
        count=0,
    )
    time.sleep(0.08)
    assert 0x111 in ctrl._bus.sent
    assert 0x421 in ctrl._bus.sent


def test_rebuild_bus_same_channel(ctrl) -> None:
    old_bus = ctrl._bus
    ctrl.rebuild_bus()
    time.sleep(0.05)
    assert ctrl._channel == "can0"
    assert ctrl._bus is not old_bus
    assert old_bus.shutdown_calls >= 1


def test_remote_alive_from_0x241(ctrl) -> None:
    import can
    from bunker_mini.protocol import CanId

    assert ctrl.remote_alive is False
    ctrl._handle_message(can.Message(
        arbitration_id=int(CanId.REMOTE_CONTROL),
        data=bytes([0b00000100, 0, 0, 0, 0, 0, 0, 1]),
        is_extended_id=False,
    ))
    assert ctrl.remote_alive is True
    assert ctrl.remote_swb_command is False


def test_switch_channel_restarts_loops(ctrl) -> None:
    ctrl.switch_channel("can1")
    time.sleep(0.05)
    # 切换后 RX/TX 线程应存活（重新启动过）
    assert ctrl._rx_thread is not None and ctrl._rx_thread.is_alive()
    assert ctrl._tx_thread is not None and ctrl._tx_thread.is_alive()
