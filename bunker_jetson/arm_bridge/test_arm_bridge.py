"""arm_bridge 单元测试：抓取等待语义 / 多通道 / 超时 / 取消 / 规格解析。"""

import threading
import time

import pytest

from arm_bridge import ArmBridge, GraspWaitResult, parse_arm_signal_spec
from arm_bridge.arm_bridge import ArmFeedbackBackend
from arm_bridge.feedback import ArmFeedbackEvent
from arm_bridge.file_backend import FileArmBackend
from arm_bridge.ws_backend import WsArmBackend


# ----------------------------------------------------------------------
# 假后端
# ----------------------------------------------------------------------


class _InstantBackend(ArmFeedbackBackend):
    """start 后第一次轮询即报告抓取完成。"""

    name = "instant"

    def __init__(self) -> None:
        self.started = 0
        self.stopped = 0

    def start(self) -> None:
        self.started += 1

    def stop(self) -> None:
        self.stopped += 1

    def wait_event(self, timeout_s: float):
        return ArmFeedbackEvent.GRASPED


class _SilentBackend(ArmFeedbackBackend):
    """永不报告完成。"""

    name = "silent"

    def __init__(self) -> None:
        self.started = 0
        self.stopped = 0

    def start(self) -> None:
        self.started += 1

    def stop(self) -> None:
        self.stopped += 1

    def wait_event(self, timeout_s: float):
        time.sleep(min(timeout_s, 0.05))
        return None


# ----------------------------------------------------------------------
# wait_grasp 语义
# ----------------------------------------------------------------------


def test_wait_grasp_returns_grasped_when_backend_signals():
    backend = _InstantBackend()
    bridge = ArmBridge([backend], wait_timeout_s=10.0)
    bridge.start()
    try:
        assert bridge.wait_grasp() == GraspWaitResult.GRASPED
    finally:
        bridge.stop()
    assert backend.stopped >= 1


def test_wait_grasp_timeout():
    bridge = ArmBridge([_SilentBackend()], wait_timeout_s=0.5)
    bridge.start()
    try:
        t0 = time.monotonic()
        assert bridge.wait_grasp() == GraspWaitResult.TIMEOUT
        assert time.monotonic() - t0 < 3.0
    finally:
        bridge.stop()


def test_wait_grasp_cancelled_via_cancel_event():
    bridge = ArmBridge([_SilentBackend()], wait_timeout_s=30.0)
    bridge.start()
    cancel = threading.Event()
    result_holder = []

    def _wait():
        result_holder.append(bridge.wait_grasp(cancel=cancel))

    thread = threading.Thread(target=_wait, daemon=True)
    thread.start()
    time.sleep(0.2)
    cancel.set()
    thread.join(timeout=2.0)
    bridge.stop()
    assert result_holder == [GraspWaitResult.CANCELLED]


def test_wait_grasp_disabled_without_backends():
    bridge = ArmBridge([], wait_timeout_s=10.0)
    assert bridge.wait_grasp() == GraspWaitResult.DISABLED


def test_external_notify_grasped_via_ws_backend():
    ws = WsArmBackend()
    bridge = ArmBridge([ws, _SilentBackend()], wait_timeout_s=10.0)
    bridge.start()

    def _notify():
        time.sleep(0.2)
        bridge.notify_grasped(source="ws")

    threading.Thread(target=_notify, daemon=True).start()
    try:
        assert bridge.wait_grasp() == GraspWaitResult.GRASPED
    finally:
        bridge.stop()


# ----------------------------------------------------------------------
# 规格解析
# ----------------------------------------------------------------------


def test_parse_spec_empty_or_none_disables():
    assert parse_arm_signal_spec("") == []
    assert parse_arm_signal_spec("none") == []


def test_parse_spec_ws_only():
    backends = parse_arm_signal_spec("ws")
    assert [b.name for b in backends] == ["ws"]


def test_parse_spec_multiple_channels():
    backends = parse_arm_signal_spec("ws,file:/tmp/grasp.sig")
    assert [b.name for b in backends] == ["ws", "file"]


def test_parse_spec_can_with_frame_id():
    backends = parse_arm_signal_spec("ws,can:can1:0x3A1")
    assert [b.name for b in backends] == ["ws", "can"]
    can_backend = backends[1]
    assert can_backend._frame_id == 0x3A1
    assert can_backend._channel == "can1"


def test_parse_spec_gpio_polarity():
    backends = parse_arm_signal_spec("gpio:18:low")
    assert [b.name for b in backends] == ["gpio"]
    gpio = backends[0]
    assert gpio._pin == 18
    assert gpio._active_high is False


def test_parse_spec_unknown_segment_skipped():
    backends = parse_arm_signal_spec("ws,bogus:zzz,file:/tmp/x")
    assert [b.name for b in backends] == ["ws", "file"]


# ----------------------------------------------------------------------
# 文件后端
# ----------------------------------------------------------------------


def test_file_backend_reads_completion(tmp_path):
    sig = tmp_path / "grasp.sig"
    backend = FileArmBackend(str(sig), poll_s=0.02)
    backend.start()
    result_holder = []

    def _wait():
        result_holder.append(backend.wait_event(5.0))

    thread = threading.Thread(target=_wait, daemon=True)
    thread.start()
    time.sleep(0.1)
    sig.write_text("grasped\n", encoding="utf-8")
    thread.join(timeout=2.0)
    backend.stop()
    assert result_holder == [ArmFeedbackEvent.GRASPED]


def test_file_backend_ignores_other_content(tmp_path):
    sig = tmp_path / "grasp.sig"
    sig.write_text("busy\n", encoding="utf-8")
    backend = FileArmBackend(str(sig), poll_s=0.02)
    backend.start()
    try:
        assert backend.wait_event(0.2) is None
    finally:
        backend.stop()
