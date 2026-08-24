"""机械臂「抓取完成」等待集成测试。

覆盖 agent 侧的抓取等待钩子（_wait_arm_grasp）、grasp_done 指令、以及
wait 与超时/取消的语义。默认（未配置信号通道）行为保持旧版：就位即返回。
"""

import json
import threading
import time

import pytest

import bunker_mini.agent as agent_mod
from bunker_mini.agent import AgentState, BunkerMiniAgent, Command
from bunker_mini.navigator import Pose2D
from bunker_mini.obstacle import ObstacleGuard, ObstaclePolicy
from bunker_mini.occupancy import OccupancyGrid
from bunker_mini.global_planner import GlobalPlanner
from arm_bridge import GraspWaitResult


class _Ctrl:
    def __init__(self) -> None:
        self.calls: list = []
        self.stops = 0
        self.latest_status = None
        self.latest_motion = None
        self.latest_bms = None
        self.latest_odometer = None

    def set_velocity(self, v, w):
        self.calls.append(("ctrl", v, w))

    def stop_motion(self):
        self.stops += 1

    def stop(self):
        pass


class _FakeLidar:
    is_receiving = True
    frame_count = 42
    packet_count = 1000
    bad_packet_count = 2
    using_difop_calibration = False

    @property
    def latest_frame(self):
        return type("F", (), {"points": []})()

    def stop(self):
        pass

    def nearest_in_range(self, *args, **kwargs):
        return 1.5

    def sector_points(self):
        return [(0.0, 1.5), (90.0, 2.0)]

    def point_cloud(self, max_points=3000, max_range_m=60.0):
        pts = [[0.1 * i, 0.2 * i, 0.3, 100] for i in range(1, 5)]
        return {"online": True, "pointCount": 4, "points": pts}

    @property
    def terrain(self):
        return None


class _Nav:
    def __init__(self) -> None:
        self.pose = Pose2D(3.0, 4.0, 1.0)
        self.is_navigating = False
        self.reset_calls = 0
        self.stopped = 0

    def stop(self):
        self.stopped += 1

    def reset_pose(self):
        self.reset_calls += 1
        self.pose = Pose2D(0.0, 0.0, 0.0)

    def goto(self, *args, **kwargs):
        return True


class _FakeWs:
    def __init__(self) -> None:
        self.sent: list[str] = []

    def send(self, raw: str) -> None:
        self.sent.append(raw)

    def close(self) -> None:
        pass


def _make_agent(monkeypatch, arm_grasp_signal: str = "",
                arm_wait_timeout_s: float = 1.0) -> BunkerMiniAgent:
    def _fake_resolve(channel, interface, *, allow_auto_channel=True):
        return "0", "virtual"

    monkeypatch.setattr(agent_mod, "resolve_can_config", _fake_resolve)
    a = BunkerMiniAgent(
        ws_url="ws://127.0.0.1:1/test",
        device_id="TEST-ARM",
        bind_code="TEST-BIND-xxxx",
        arm_grasp_signal=arm_grasp_signal,
        arm_wait_timeout_s=arm_wait_timeout_s,
    )
    a._controller = _Ctrl()
    a._lidar = _FakeLidar()
    a._navigator = _Nav()
    a._guard = ObstacleGuard(a._lidar, ObstaclePolicy())
    a._occ_grid = OccupancyGrid()
    a._planner = GlobalPlanner(a._occ_grid)
    return a


@pytest.fixture
def arm_agent(monkeypatch) -> BunkerMiniAgent:
    a = _make_agent(monkeypatch, arm_grasp_signal="ws",
                    arm_wait_timeout_s=1.0)
    yield a
    a.stop()
    a._cleanup()


@pytest.fixture
def plain_agent(monkeypatch) -> BunkerMiniAgent:
    """未配置信号通道 → 机械臂等待关闭（旧行为）。"""
    a = _make_agent(monkeypatch, arm_grasp_signal="", arm_wait_timeout_s=1.0)
    yield a
    a.stop()
    a._cleanup()


# ----------------------------------------------------------------------
# _wait_arm_grasp 语义
# ----------------------------------------------------------------------


def test_wait_disabled_without_signal(plain_agent):
    assert plain_agent._arm_bridge is None or not plain_agent._arm_bridge.enabled
    assert plain_agent._wait_arm_grasp(threading.Event(), "rock") == "disabled"


def test_wait_early_ws_signal_returns_grasped(arm_agent):
    """信号在进入等待前已到达（如操作员提前下发 gd）→ 等待立即返回。"""
    events = []
    arm_agent._send_event = lambda e, m: events.append((e, m))
    arm_agent._handle_grasp_done(Command(action="grasp_done"))
    assert any(e == "grasp_done" for e, _ in events)
    result = arm_agent._wait_arm_grasp(threading.Event(), "rock")
    assert result == "grasped"
    assert any(e == "grasped" for e, _ in events)
    # mission 状态流转：holding → grasped
    assert arm_agent._mission.get("status") == "grasped"


def test_wait_timeout_when_no_signal(monkeypatch):
    agent = _make_agent(monkeypatch, arm_grasp_signal="ws",
                        arm_wait_timeout_s=0.3)
    try:
        t0 = time.monotonic()
        result = agent._wait_arm_grasp(threading.Event(), "rock")
        assert result == "timeout"
        assert time.monotonic() - t0 < 3.0
        # _wait_arm_grasp 只负责等待语义；timeout 后的 failed 状态由
        # _find_object_loop 调用方更新（保持等待逻辑纯净、可复用）
        assert agent._mission.get("status") == "holding"
    finally:
        agent.stop()
        agent._cleanup()


def test_wait_cancelled_via_stop_event(arm_agent):
    stop = threading.Event()
    result_holder = []

    def _wait():
        result_holder.append(arm_agent._wait_arm_grasp(stop, "rock"))

    thread = threading.Thread(target=_wait, daemon=True)
    thread.start()
    time.sleep(0.2)
    stop.set()
    thread.join(timeout=2.0)
    assert result_holder == ["cancelled"]


# ----------------------------------------------------------------------
# grasp_done 指令
# ----------------------------------------------------------------------


def test_grasp_done_command_routes_and_emits(arm_agent):
    events = []
    arm_agent._send_event = lambda e, m: events.append((e, m))
    arm_agent._handle_command(Command(action="grasp_done"))
    assert any(e == "grasp_done" for e, _ in events)
    # 桥接层已置位 → 等待会立即返回
    assert arm_agent._arm_bridge.wait_grasp(timeout_s=0.1) \
        == GraspWaitResult.GRASPED


def test_grasp_done_without_bridge_is_harmless(plain_agent):
    events = []
    plain_agent._send_event = lambda e, m: events.append((e, m))
    plain_agent._handle_command(Command(action="grasp_done"))
    assert any(e == "grasp_done" for e, _ in events)


# ----------------------------------------------------------------------
# 状态上报：holding / grasped 可见
# ----------------------------------------------------------------------


def test_push_state_shows_holding_and_grasped(arm_agent):
    arm_agent._state = AgentState.ONLINE
    fake_ws = _FakeWs()
    arm_agent._ws = fake_ws

    stop = threading.Event()
    result_holder = []

    def _wait():
        result_holder.append(arm_agent._wait_arm_grasp(stop, "rock"))

    thread = threading.Thread(target=_wait, daemon=True)
    thread.start()
    time.sleep(0.15)  # 等待已进入 holding
    arm_agent._push_state()
    payload = json.loads(fake_ws.sent[-1])["payload"]
    assert payload["mission"]["status"] == "holding"

    arm_agent._handle_command(Command(action="grasp_done"))
    thread.join(timeout=2.0)
    assert result_holder == ["grasped"]
    arm_agent._push_state()
    payload = json.loads(fake_ws.sent[-1])["payload"]
    assert payload["mission"]["status"] == "grasped"
