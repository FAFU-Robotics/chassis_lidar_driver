"""BunkerMiniController 统一速度斜坡（加速度限制）测试。

背景：所有驱动者（导航/避障/回放/patrol/kb move）最终都汇入 set_velocity。
旧实现直接覆盖 _command，速度是阶跃的——起步 0→v、避障解除后 0→v_cmd、
转向切换都是瞬间跳变，高速下即「爆发」（与 kb 长按警示同构）。

修复：set_velocity 应用加速度斜坡（加速柔和、减速陡），set_velocity_now
直通（ESTOP / 看门狗立即停车）。
"""

import time

import pytest

import bunker_mini.controller as controller_mod
from bunker_mini.controller import (
    MAX_ACCEL_M_S2,
    MAX_DECEL_M_S2,
    BunkerMiniController,
)


class _FakeBus:
    def __init__(self) -> None:
        self.sent: list = []

    def recv(self, timeout=None):
        # 模拟真实 CAN 接收阻塞（避免 RX 线程忙转耗尽 CPU）
        time.sleep(0.02)
        return None

    def send(self, msg) -> None:
        self.sent.append(msg)

    def shutdown(self) -> None:
        pass


@pytest.fixture
def ctrl(monkeypatch) -> BunkerMiniController:
    monkeypatch.setattr(controller_mod, "create_can_bus", lambda *a, **k: _FakeBus())
    c = BunkerMiniController(channel="can0", interface="socketcan")
    c.start()
    time.sleep(0.05)
    yield c
    c.stop()


def _read_cmd(ctrl) -> tuple[float, float]:
    """读出当前 _command 的 (v m/s, w rad/s)。"""
    return (
        ctrl._command.linear_velocity_mm_s / 1000.0,
        ctrl._command.angular_velocity_mrad_s / 1000.0,
    )


def _drive_to(ctrl, v, w, steps=60, dt=0.02):
    """以 20ms 间隔反复下发目标速度，模拟真实控制循环。

    0→0.5 m/s 需 50 步（加速 0.5 m/s²）；0.5→0 需 17 步（减速 1.5 m/s²）。
    默认 60 步足够覆盖，保持测试总时长 ~1.2s/段。
    """
    for _ in range(steps):
        ctrl.set_velocity(v, w)
        time.sleep(dt)


def test_step_up_is_ramped_not_instant(ctrl) -> None:
    """0 → 0.5 m/s 应斜坡爬升，第一拍远小于目标（无阶跃爆发）。"""
    ctrl.set_velocity(0.5, 0.0)
    v, _ = _read_cmd(ctrl)
    assert 0.0 < v < 0.5, f"起步应为斜坡爬升，实际第一拍 {v}（出现阶跃爆发）"
    # 连续多次调用（模拟导航循环）最终到达目标
    _drive_to(ctrl, 0.5, 0.0, steps=60)
    v, _ = _read_cmd(ctrl)
    assert abs(v - 0.5) < 0.01, f"斜坡后应达到目标 0.5，实际 {v}"


def test_ramp_step_magnitude(ctrl) -> None:
    """单次斜坡步长 ≈ accel * dt（20ms 步进 ≈ 0.01 m/s）。"""
    ctrl.set_velocity(0.5, 0.0)
    v, _ = _read_cmd(ctrl)
    time.sleep(0.02)
    ctrl.set_velocity(0.5, 0.0)
    v2, _ = _read_cmd(ctrl)
    step = v2 - v
    assert 0.0 < step <= MAX_ACCEL_M_S2 * 0.05 + 1e-6, f"斜坡步长异常: {step}"


def test_step_down_is_ramped_not_instant(ctrl) -> None:
    """0.5 → 0 应斜坡减速，第一拍 > 0（无阶跃急停）。"""
    _drive_to(ctrl, 0.5, 0.0, steps=60)
    ctrl.set_velocity(0.0, 0.0)
    v, _ = _read_cmd(ctrl)
    assert 0.0 < v < 0.5, f"减速应斜坡（非阶跃），实际 {v}"
    _drive_to(ctrl, 0.0, 0.0, steps=30)
    v, _ = _read_cmd(ctrl)
    assert abs(v) < 0.01, f"斜坡后应完全停车，实际 {v}"


def test_stop_motion_immediate(ctrl) -> None:
    """stop_motion（ESTOP / 看门狗用）应直通立即停车，不走斜坡。"""
    _drive_to(ctrl, 0.5, 0.0, steps=60)
    ctrl.stop_motion()
    v, _ = _read_cmd(ctrl)
    assert v == 0.0, "stop_motion 应直通立即停车"


def test_set_velocity_now_immediate(ctrl) -> None:
    """set_velocity_now 直通：从斜坡中途一步跳到目标值。"""
    _drive_to(ctrl, 0.5, 0.0, steps=10)
    ctrl.set_velocity_now(0.3, 0.2)
    v, w = _read_cmd(ctrl)
    assert abs(v - 0.3) < 0.001 and abs(w - 0.2) < 0.001


def test_ramp_state_reset_after_estop(ctrl) -> None:
    """急停后斜坡基准归零，下一次起步仍从 0 平滑爬升。"""
    _drive_to(ctrl, 0.5, 0.0, steps=60)
    ctrl.stop_motion()
    time.sleep(0.02)  # 模拟急停后一个控制周期再恢复驾驶
    ctrl.set_velocity(0.5, 0.0)
    v, _ = _read_cmd(ctrl)
    assert 0.0 < v < 0.5, "急停后再起步应重新斜坡（无残留速度）"


def test_controller_switch_still_works(ctrl) -> None:
    """斜坡不影响通道热切换。"""
    ctrl.set_velocity(0.5, 0.0)
    ctrl.switch_channel("can1")
    time.sleep(0.05)
    assert ctrl._channel == "can1"
