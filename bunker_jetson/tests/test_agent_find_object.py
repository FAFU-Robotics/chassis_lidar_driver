"""Agent find_object 自主探路编排（recon 巡游 + 轨迹返回）测试。

覆盖 agent._recon_search / _return_home 的状态机与事件流：
  * 巡游中检测到目标 → 返回 (est, track)，轨迹被自动记录并保存
  * 巡游超时未找到 → 返回 (None, track)，走轨迹返回 + go_home 兜底
  * 无 recorder / 无轨迹时 _return_home 直接 go_home
"""

import math
import threading
import time

import pytest

import bunker_mini.agent as agent_mod
from bunker_mini.agent import BunkerMiniAgent, Command
from bunker_mini.navigator import Pose2D
from bunker_mini.tracker import Track, Waypoint


class _Est:
    """Dummy TargetEstimate 替身（满足 _confirm_target 读取的字段）。"""

    def __init__(self, distance_m: float = 1.5, bearing_deg: float = 0.0,
                 radius_estimate_m: float = 0.0) -> None:
        self.distance_m = distance_m
        self.bearing_deg = bearing_deg
        self.radius_estimate_m = radius_estimate_m
        self.center_distance_m = distance_m + radius_estimate_m


class _FakeDetector:
    def __init__(self, hits_from: int | None = None,
                 hits_at: list[int] | None = None) -> None:
        self._hits_from = hits_from
        self._hits_at = hits_at or []
        self.calls = 0

    def detect(self, points):
        self.calls += 1
        if self._hits_from is not None and self.calls >= self._hits_from:
            return [_Est()]  # 之后持续命中（多帧确认需要连续帧）
        if self.calls in self._hits_at:
            return [_Est()]
        return []


class _FakeLidar:
    def __init__(self) -> None:
        self.is_receiving = True
        self.frame = type("F", (), {"points": [1, 2, 3]})()

    @property
    def latest_frame(self):
        return self.frame

    def stop(self):
        pass


class _FakePatrol:
    def __init__(self) -> None:
        self.updates = 0
        self.stopped = False
        self.started = 0

    def start(self):
        self.started += 1
        self.stopped = False

    def update(self):
        self.updates += 1
        return 0.25, 0.0

    def stop(self):
        self.stopped = True


class _FakeController:
    def __init__(self) -> None:
        self.motion = type("M", (), {"linear_velocity_m_s": 0.0,
                                     "angular_velocity_rad_s": 0.0})()
        self.odometer = type("O", (), {"left_wheel_mm": 0, "right_wheel_mm": 0})()

    def set_velocity(self, v, w):
        pass

    def stop_motion(self):
        pass

    def stop(self):
        pass


class _FakeNavigator:
    """导航替身：goto 后台线程自动完成（模拟真实 Navigator 线程结束）。"""

    def __init__(self, latency_s: float = 0.01) -> None:
        self.pose = type("P", (), {"x": 0.0, "y": 0.0, "yaw": 0.0})()
        self._nav = False
        self._latency = latency_s
        self._goals: list[tuple[float, float]] = []

    @property
    def is_navigating(self):
        return self._nav

    @property
    def goals(self):
        return self._goals

    def goto(self, x, y, *, on_arrived=None, on_abort=None, **_kw):
        self._goals.append((x, y))
        self._nav = True

        def _finish():
            time.sleep(self._latency)
            self._nav = False

        threading.Thread(target=_finish, daemon=True).start()
        return True

    def stop(self):
        self._nav = False


class _FakeRecorder:
    def __init__(self) -> None:
        self.recording = False
        self.started_name = ""
        self._track = Track(
            name="recon_1",
            created_at="t",
            total_duration_s=1.0,
            waypoints=[
                Waypoint(0.0, 0, 0, 0.0, 0.0),
                Waypoint(1.0, 100, 100, 0.25, 0.0),
            ],
        )

    @property
    def is_recording(self):
        return self.recording

    def start(self, name=""):
        self.recording = True
        self.started_name = name

    def stop(self):
        self.recording = False
        return self._track


class _FakePlayer:
    def __init__(self, complete_immediately: bool = False) -> None:
        self.complete_immediately = complete_immediately
        self.played: list[Track] = []
        self._playing = False
        self.saved: list[str] = []

    @property
    def is_playing(self):
        return self._playing

    def play_async(self, track, *, on_complete=None, reverse=False, bypass_guard=False):
        self.played.append(track)
        if self.complete_immediately:
            # 模拟轨迹播放立即成功完成
            if on_complete is not None:
                on_complete(True)
            return
        self._playing = True
        self._on_complete = on_complete

    def finish(self):
        """模拟轨迹播放成功走完。"""
        self._playing = False
        if getattr(self, "_on_complete", None) is not None:
            self._on_complete(True)
            self._on_complete = None

    def stop(self):
        self._playing = False
        self._on_complete = None

    def save_track(self, track):
        self.saved.append(track.name)


@pytest.fixture
def agent(monkeypatch) -> BunkerMiniAgent:
    def _fake_resolve(channel, interface, *, allow_auto_channel=True):
        return "0", "virtual"

    monkeypatch.setattr(agent_mod, "resolve_can_config", _fake_resolve)
    a = BunkerMiniAgent(
        ws_url="ws://127.0.0.1:1/test",
        device_id="TEST-01",
        bind_code="TEST-BIND-xxxx",
    )
    a._controller = None
    yield a
    a.stop()
    a._cleanup()


def _arm_agent(agent, hits_from=None, hits_at=None):
    """Wire up fakes so find_object 编排可运行。"""
    agent._detector = _FakeDetector(hits_from=hits_from, hits_at=hits_at)
    agent._lidar = _FakeLidar()
    agent._patrol = _FakePatrol()
    agent._controller = _FakeController()
    agent._navigator = _FakeNavigator()
    agent._recorder = _FakeRecorder()
    agent._player = _FakePlayer()
    agent._mission_stop_event = threading.Event()
    return agent


def test_recon_finds_target_and_records_track(agent):
    """巡游第 2 帧检测到目标 → 返回 est；录制不在此定格，
    由调用方在导航/对接完成后定格完整轨迹并保存。"""
    agent = _arm_agent(agent, hits_from=2)
    agent._initial_sweep_enabled = False  # 走巡游路径检测（本测试覆盖巡游分支）
    est, track = agent._recon_search(
        "rock", agent._detector, agent._lidar,
        agent._controller, agent._patrol, agent._mission_stop_event,
    )
    assert est is not None
    # 找到目标 → 录制继续（轨迹要延伸到目标点），track 由后续 finalize 给出
    assert track is None
    assert agent._recorder.is_recording is True
    assert agent._player.saved == []
    assert agent._patrol.started >= 1             # 每次探路必须重新 start
    assert agent._patrol.stopped                   # 巡游已停止
    assert agent._patrol.updates >= 1

    # 导航/对接完成后定格完整轨迹并落盘
    full = agent._finalize_recon_track()
    assert full is not None
    assert full.name == "recon_1"
    assert agent._recorder.is_recording is False   # 已停止录制
    assert agent._player.saved == ["recon_1"]      # 探路轨迹已落盘


def test_recon_timeout_returns_none_then_return_home(agent):
    """巡游超时未找到目标 → (None, track)，随后走轨迹返回。"""
    agent = _arm_agent(agent)  # 永远检测不到
    agent._initial_sweep_enabled = False  # 跳过出发前扫描，直接巡游
    agent._recon_max_duration_s = 0.05     # 快速超时
    # 轨迹播放设为"立即完成"，模拟播放成功走完
    agent._player = _FakePlayer(complete_immediately=True)
    est, track = agent._recon_search(
        "rock", agent._detector, agent._lidar,
        agent._controller, agent._patrol, agent._mission_stop_event,
    )
    assert est is None
    assert track is not None  # 即便没找到也保存了轨迹

    # 进入返回阶段：优先沿轨迹反向返回
    agent._return_home("rock", track, agent._mission_stop_event)
    assert agent._player.played
    assert agent._player.played[0].waypoints[0].t == pytest.approx(0.0)
    assert agent._player.played[0].waypoints[0].v < 0.0  # 反向速度
    assert agent._player.played[0].total_duration_s == pytest.approx(1.0)
    # 轨迹正常完成 → 不走 goto 兜底
    assert agent._navigator.goals == []


def test_return_home_falls_back_to_goto_without_track(agent):
    """无轨迹时 _return_home 直接 go_home（goto 0,0）。"""
    agent = _arm_agent(agent)
    nav = agent._navigator
    agent._return_home("rock", None, agent._mission_stop_event)
    # 等待导航自动完成
    deadline = time.monotonic() + 2.0
    while nav.is_navigating and time.monotonic() < deadline:
        time.sleep(0.01)
    assert nav.goals == [(0.0, 0.0)]  # 触发了 goto(0,0)


def test_recon_no_recorder_still_finds_target(agent):
    """recorder 不存在（如纯视觉快速模式）也不影响检测。"""
    agent = _arm_agent(agent, hits_from=1)
    agent._recorder = None
    est, track = agent._recon_search(
        "rock", agent._detector, agent._lidar,
        agent._controller, agent._patrol, agent._mission_stop_event,
    )
    assert est is not None
    assert track is None


class _AlignNav:
    """导航替身：pose 随原地转向更新（模拟真实 0x311 积分）。"""

    def __init__(self) -> None:
        self.pose = Pose2D(0.0, 0.0, 0.0)

    def turn(self, w: float) -> None:
        # 每控制周期 50ms 的角速度积分
        self.pose = Pose2D(self.pose.x, self.pose.y,
                           self.pose.yaw + w * 0.05)


class _AlignCtrl(_FakeController):
    def __init__(self, nav: _AlignNav) -> None:
        super().__init__()
        self._nav = nav
        self.turn_cmds: list[float] = []

    def set_velocity(self, v: float, w: float) -> None:
        super().set_velocity(v, w)
        if v == 0.0 and w != 0.0:
            self.turn_cmds.append(w)
            self._nav.turn(w)


def test_return_heading_alignment_turns_in_place(agent):
    """返回前航向偏差 >20° → 原地转向对齐（下发 (0, ±w)）后再放行。"""
    agent = _arm_agent(agent)
    nav = _AlignNav()
    agent._navigator = nav
    agent._controller = _AlignCtrl(nav)
    agent._recon_start_pose = Pose2D(0.0, 0.0, 0.0)
    # 轨迹末端相对航向 45°，起点航向 0° → 期望返回航向 45°，当前 0°
    agent._track_end_pose = lambda t: Pose2D(0.1, 0.0, math.radians(45.0))
    track = Track(name="t", created_at="c", total_duration_s=1.0,
                  waypoints=[Waypoint(0.0, 0, 0, 0.2, 0.0)])
    assert agent._align_return_heading(
        track, agent._mission_stop_event) is True
    assert any(w > 0 for w in agent._controller.turn_cmds)  # 左转对齐


def test_return_position_mismatch_falls_back_to_goto(agent):
    """当前位置偏离轨迹终点 >0.5m → 放弃沿轨迹，回退 go_home。"""
    agent = _arm_agent(agent)
    nav = agent._navigator
    agent._recon_start_pose = Pose2D(0.0, 0.0, 0.0)
    agent._track_end_pose = lambda t: Pose2D(3.0, 0.0, 0.0)  # 终点 3m 外
    track = Track(name="t", created_at="c", total_duration_s=1.0,
                  waypoints=[Waypoint(0.0, 0, 0, 0.2, 0.0)])
    # navigator pose (0,0,0) → 位置偏差 3m，对齐判定拒绝
    assert agent._align_return_heading(
        track, agent._mission_stop_event) is False
    # 无轨迹可用时 _return_home 走 goto(0,0) 兜底
    agent._player = _FakePlayer()
    agent._return_home("rock", track, agent._mission_stop_event)
    deadline = time.monotonic() + 2.0
    while nav.is_navigating and time.monotonic() < deadline:
        time.sleep(0.01)
    assert nav.goals == [(0.0, 0.0)]


def test_standoff_goal_pulls_back_from_object_center(agent):
    """导航终点应停在目标前方，而不是开进中心。"""
    sx, sy = agent._standoff_goal(0.0, 0.0, 2.0, 0.0, 0.5)
    assert sx == pytest.approx(1.5)
    assert sy == pytest.approx(0.0)
    sx, sy = agent._standoff_goal(0.0, 0.0, 0.3, 0.0, 0.5)
    assert sx == pytest.approx(0.0)
    assert sy == pytest.approx(0.0)


def test_second_recon_restarts_patrol(agent):
    """上一轮 patrol.stop() 后，第二次探路必须再 start，否则会原地发呆。"""
    agent = _arm_agent(agent, hits_from=2)
    agent._initial_sweep_enabled = False
    agent._recon_search(
        "rock", agent._detector, agent._lidar,
        agent._controller, agent._patrol, agent._mission_stop_event,
    )
    assert agent._patrol.started == 1
    agent._detector = _FakeDetector(hits_from=2)
    agent._mission_stop_event = threading.Event()
    agent._recon_search(
        "rock", agent._detector, agent._lidar,
        agent._controller, agent._patrol, agent._mission_stop_event,
    )
    assert agent._patrol.started == 2


def test_fail_and_return_goes_home_and_keeps_failed_status(agent):
    """导航/对接失败必须返航，且终态仍是 failed 而不是 done。"""
    agent = _arm_agent(agent)
    agent._player = _FakePlayer(complete_immediately=True)
    track = Track(
        name="recon_fail", created_at="t", total_duration_s=1.0,
        waypoints=[Waypoint(0.0, 0, 0, 0.2, 0.0),
                   Waypoint(1.0, 100, 100, 0.25, 0.0)],
    )
    agent._recon_track = track
    agent._recon_start_pose = Pose2D(0.0, 0.0, 0.0)
    agent._fail_and_return("rock", "导航失败", track, agent._mission_stop_event)
    deadline = time.monotonic() + 2.0
    while agent._player.is_playing and time.monotonic() < deadline:
        time.sleep(0.01)
    assert agent._player.played
    assert agent._mission["status"] == "failed"


def test_cancel_uses_saved_recon_track(agent):
    """探路中 cancel：录制可能已定格，必须用 _recon_track 返航。"""
    agent = _arm_agent(agent)
    agent._player = _FakePlayer(complete_immediately=True)
    agent._recorder.recording = False
    track = Track(
        name="recon_saved", created_at="t", total_duration_s=1.0,
        waypoints=[Waypoint(0.0, 0, 0, 0.2, 0.0),
                   Waypoint(1.0, 100, 100, 0.25, 0.0)],
    )
    agent._recon_track = track
    agent._recon_start_pose = Pose2D(0.0, 0.0, 0.0)
    agent._handle_cancel(Command(action="cancel", name="rock"))
    deadline = time.monotonic() + 2.0
    while not agent._player.played and time.monotonic() < deadline:
        time.sleep(0.01)
    assert agent._player.played


def test_cancel_without_track_still_goes_home(agent):
    """无探路轨迹时 cancel 也必须直线返回，不能停在溶洞里。"""
    agent = _arm_agent(agent)
    agent._recorder.recording = False
    agent._recon_track = None
    nav = agent._navigator
    agent._handle_cancel(Command(action="cancel", name="rock"))
    deadline = time.monotonic() + 2.0
    while nav.is_navigating and time.monotonic() < deadline:
        time.sleep(0.01)
    assert nav.goals == [(0.0, 0.0)]
