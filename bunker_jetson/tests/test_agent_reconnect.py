"""Agent WebSocket disconnect handling regression tests.

背景：模拟云端（mock_cloud）被退出/强杀后，车侧代理曾疯狂刷屏
"--- Logging error ---" 与 traceback，看起来像"崩溃"：
  1. ``_on_close`` 的 ``logger.info("code=%d ...", close_status_code, ...)``
     在非正常断开时 close_status_code/close_msg 为 None，``%d`` 格式化
     抛 ``TypeError``，每次断线/重连都触发一次 Logging error 刷屏。
  2. 断线时 ``_send_event_inline`` 用已关闭的连接 ``ws.send()`` 抛
     ``WebSocketConnectionClosedException``，原实现打完整 traceback。
这两个测试防止上述问题回归。
"""

import threading
import time

import pytest
import websocket

import bunker_mini.agent as agent_mod
from bunker_mini.agent import AgentState, BunkerMiniAgent


class _FakeWs:
    """Minimal stand-in for websocket.WebSocketApp."""

    def __init__(self, fail_send: bool = False) -> None:
        self._fail_send = fail_send
        self.sent: list[str] = []

    def send(self, raw: str) -> None:
        if self._fail_send:
            raise websocket.WebSocketConnectionClosedException("Connection is already closed.")
        self.sent.append(raw)

    def close(self) -> None:
        pass


@pytest.fixture
def agent(monkeypatch) -> BunkerMiniAgent:
    # 构造阶段会走 resolve_can_config；测试环境无需真实 CAN，
    # 用 virtual 总线替身，避免依赖硬件。
    def _fake_resolve(channel, interface, *, allow_auto_channel=True):
        return "0", "virtual"

    monkeypatch.setattr(agent_mod, "resolve_can_config", _fake_resolve)
    a = BunkerMiniAgent(
        ws_url="ws://127.0.0.1:1/test",
        device_id="TEST-01",
        bind_code="TEST-BIND-xxxx",
    )
    # 不触发真实网络/CAN
    a._controller = None
    yield a
    a.stop()
    a._cleanup()


def test_on_close_none_code_does_not_crash_logging(agent, caplog):
    """非正常断开（code=None, msg=None）时 _on_close 不抛日志格式化异常。"""
    fake = _FakeWs(fail_send=True)
    with agent._lock:
        agent._state = AgentState.ONLINE
        agent._ws = fake
    with caplog.at_level("INFO"):
        agent._on_close(fake, None, None, agent._session_id)
    # 状态应回落到 DISCONNECTED
    assert agent.state == AgentState.DISCONNECTED
    # 不应出现 logging 自身的错误（%d 格式化 TypeError）
    assert not any("Logging error" in r.message for r in caplog.records)
    assert not any("TypeError" in r.message for r in caplog.records)


def test_on_close_send_failure_is_swallowed_not_traceback(agent, caplog):
    """断线时 OFFLINE 事件发送失败应被静默处理，不产生异常 traceback。"""
    fake = _FakeWs(fail_send=True)
    with agent._lock:
        agent._state = AgentState.ONLINE
        agent._ws = fake
    with caplog.at_level("DEBUG"):
        agent._on_close(fake, None, None, agent._session_id)
    assert agent.state == AgentState.DISCONNECTED
    # 失败被记录为 debug，而不是 exception traceback
    exc_texts = [r.exc_text or "" for r in caplog.records]
    assert not any("Traceback" in t for t in exc_texts)


def test_on_close_stale_session_ignored(agent):
    """旧会话的 on_close 回调不应影响当前会话状态。"""
    fake = _FakeWs()
    with agent._lock:
        agent._state = AgentState.ONLINE
        agent._ws = fake
    agent._on_close(fake, None, None, agent._session_id - 1)  # 旧 session
    assert agent.state == AgentState.ONLINE  # 状态未被污染


def test_on_close_not_online_skips_send(agent, caplog):
    """非 ONLINE 状态（如重连阶段）断线时不应尝试发送 OFFLINE 事件。"""
    fake = _FakeWs()
    with agent._lock:
        agent._state = AgentState.CONNECTING
        agent._ws = fake
    with caplog.at_level("INFO"):
        agent._on_close(fake, None, None, agent._session_id)
    assert agent.state == AgentState.DISCONNECTED
    assert not fake.sent


class _FakeCtrl:
    def __init__(self) -> None:
        self.stops = 0

    def stop_motion(self) -> None:
        self.stops += 1


class _FakeNav:
    def __init__(self) -> None:
        self.is_navigating = True


def test_on_close_teleop_stops_chassis(agent):
    """纯遥控（无本地任务）断线必须立即停车。"""
    ctrl = _FakeCtrl()
    agent._controller = ctrl
    fake = _FakeWs()
    with agent._lock:
        agent._state = AgentState.ONLINE
        agent._ws = fake
    agent._on_close(fake, None, None, agent._session_id)
    assert agent.state == AgentState.DISCONNECTED
    assert ctrl.stops == 1


def test_on_close_does_not_abort_running_mission(agent):
    """find_object 任务线程活着时，WS 掉线不得停车、不得取消任务。"""
    ctrl = _FakeCtrl()
    agent._controller = ctrl
    stop = threading.Event()

    def _loop() -> None:
        while not stop.is_set():
            time.sleep(0.02)

    t = threading.Thread(target=_loop, daemon=True)
    t.start()
    agent._mission_thread = t
    agent._mission_stop_event = threading.Event()
    agent._mission = {"status": "recon", "target": "岩石A"}
    try:
        fake = _FakeWs()
        with agent._lock:
            agent._state = AgentState.ONLINE
            agent._ws = fake
            agent._pending_command = None
        agent._on_close(fake, None, None, agent._session_id)
        assert agent.state == AgentState.DISCONNECTED
        assert ctrl.stops == 0
        assert t.is_alive()
        assert not agent._mission_stop_event.is_set()
        assert agent._mission["status"] == "recon"
    finally:
        stop.set()
        t.join(timeout=1.0)


def test_on_close_does_not_abort_goto(agent):
    """独立 goto 导航中掉线，不得停车、不得 nav.stop。"""
    ctrl = _FakeCtrl()
    agent._controller = ctrl
    nav = _FakeNav()
    agent._navigator = nav
    fake = _FakeWs()
    with agent._lock:
        agent._state = AgentState.ONLINE
        agent._ws = fake
    agent._on_close(fake, None, None, agent._session_id)
    assert ctrl.stops == 0
    assert nav.is_navigating is True
