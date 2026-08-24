"""云端雷达命令（lidar_on/off/status/map）、goto 预检与导航 drive 回调测试。"""

import time

import pytest

import bunker_mini.agent as agent_mod
from bunker_mini.agent import BunkerMiniAgent, Command
from bunker_mini.navigator import Navigator, Pose2D
from bunker_mini.obstacle import ObstacleGuard, ObstaclePolicy
from bunker_mini.occupancy import OccupancyGrid
from bunker_mini.terrain import TerrainSectorResult


class _Ctrl:
    def __init__(self) -> None:
        self.calls: list = []
        self.stops = 0

    def set_velocity(self, v, w):
        self.calls.append(("ctrl", v, w))

    def stop_motion(self):
        self.stops += 1

    def stop(self):
        pass


class _Terrain:
    def all_sectors(self, step_limit=None):
        return [
            TerrainSectorResult(angle_deg=0.0),
            TerrainSectorResult(angle_deg=30.0, obstacle_distance_m=0.8,
                                max_height_m=0.1),
        ]


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
        return _Terrain()

    def terrain_sector(self, angle_deg, step_limit=None):
        return TerrainSectorResult(angle_deg=angle_deg)


class _FakeOnlineLidar:
    """启动后延迟一段时间才「在线」的雷达桩，用于验证首帧预热等待。"""

    frame_count = 0
    packet_count = 0
    bad_packet_count = 0

    def __init__(self, online_after_s: float = 0.15) -> None:
        self._started_at = 0.0
        self._online_after_s = online_after_s

    def start(self) -> None:
        self._started_at = time.monotonic()

    def stop(self) -> None:
        pass

    @property
    def is_receiving(self) -> bool:
        return self._started_at > 0.0 and (
            time.monotonic() - self._started_at >= self._online_after_s
        )


class _Nav:
    def __init__(self) -> None:
        self.pose = Pose2D(0.0, 0.0, 0.0)
        self.is_navigating = False
        self.goals: list = []
        self.stopped = 0

    def stop(self):
        self.stopped += 1

    def set_guard(self, guard):
        self.guard = guard

    def goto(self, x, y, *, on_arrived=None, on_abort=None, speed=None,
             waypoints=None, replanner=None, goal_yaw=None):
        self.goals.append((x, y, speed))
        return True


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
    a._controller = _Ctrl()
    a._lidar = _FakeLidar()
    a._navigator = _Nav()
    a._guard = ObstacleGuard(a._lidar, ObstaclePolicy())
    a._occ_grid = OccupancyGrid()
    yield a
    a.stop()
    a._cleanup()


# ----------------------------------------------------------------------
# lidar_on / lidar_off
# ----------------------------------------------------------------------


def test_lidar_on_already_online_sends_event(agent):
    events = []
    agent._send_event = lambda e, m: events.append((e, m))
    agent._handle_lidar_on(Command(action="lidar_on"))
    assert events and events[-1][0] == "lidar"


def test_lidar_off_tears_down_and_sends_event(agent):
    events = []
    agent._send_event = lambda e, m: events.append((e, m))
    agent._handle_lidar_off(Command(action="lidar_off"))
    assert agent._lidar is None
    assert agent._occ_grid is None
    assert events and events[-1][0] == "lidar"


def test_lidar_on_waits_for_first_frame_before_success(agent, monkeypatch):
    """修复误报：启动瞬间 is_receiving 必为 False，须等首帧再判成败。"""
    fake = _FakeOnlineLidar(online_after_s=0.2)
    monkeypatch.setattr(agent, "_build_lidar", lambda: fake)
    agent._lidar = None
    events = []
    agent._send_event = lambda e, m: events.append((e, m))
    agent._handle_lidar_on(Command(action="lidar_on"))
    assert events, "应有事件"
    assert events[-1][0] == "lidar"
    assert "在线" in events[-1][1]
    assert not any(e == "fault" for e, _ in events)


def test_lidar_on_reports_no_data_after_warmup(agent, monkeypatch):
    """启动成功但预热后仍收不到数据 → 具体诊断（而非笼统「检查网线」）。"""
    fake = _FakeOnlineLidar(online_after_s=10.0)
    monkeypatch.setattr(agent, "_build_lidar", lambda: fake)
    monkeypatch.setattr(agent, "_wait_lidar_online", lambda timeout_s=3.0: False)
    agent._lidar = None
    events = []
    agent._send_event = lambda e, m: events.append((e, m))
    agent._handle_lidar_on(Command(action="lidar_on"))
    assert events and events[-1][0] == "fault"
    assert "收不到数据" in events[-1][1]


def test_lidar_on_start_failure_sends_no_generic_fault(agent, monkeypatch):
    """端口被占等启动异常：具体原因已由 _start_lidar 推送，不重复笼统消息。"""
    events = []
    agent._send_event = lambda e, m: events.append((e, m))
    monkeypatch.setattr(agent, "_start_lidar", lambda: False)
    agent._handle_lidar_on(Command(action="lidar_on"))
    assert events == [], "启动失败的具体原因已由 _start_lidar 推送，不应重复"


def test_wait_lidar_online_polls_until_first_frame(agent):
    fake = _FakeOnlineLidar(online_after_s=0.15)
    fake.start()
    agent._lidar = fake
    assert agent._wait_lidar_online(timeout_s=1.0) is True


def test_wait_lidar_online_times_out(agent):
    fake = _FakeOnlineLidar(online_after_s=10.0)
    agent._lidar = fake
    t0 = time.monotonic()
    assert agent._wait_lidar_online(timeout_s=0.3) is False
    assert time.monotonic() - t0 < 2.0


# ----------------------------------------------------------------------
# lidar_status
# ----------------------------------------------------------------------


def test_lidar_status_payload(agent):
    out = []
    agent._send_event_data = lambda e, d: out.append((e, d))
    agent._handle_lidar_status(Command(action="lidar_status"))
    assert out
    e, d = out[0]
    assert e == "lidar_status"
    assert d["online"] is True
    assert d["source"] == "udp"
    assert d["frames"] == 42
    assert d["packets"] == 1000
    assert d["difop"] is False
    assert d["mount"]["yawDeg"] == 0.0
    at = d["avoidanceTest"]
    assert at["requested"] == [0.2, 0.0]
    assert at["blocked"] is False
    assert d["front"]["obstacleDistance"] == pytest.approx(1.5)


def test_lidar_status_offline_includes_diagnosis(agent):
    """离线时返回数据链路诊断：区分「0 包未到」vs「包到但解析失败」。"""
    out = []
    agent._send_event_data = lambda e, d: out.append((e, d))
    offline = type(
        "L",
        (),
        {
            "is_receiving": False,
            "frame_count": 0,
            "packet_count": 0,
            "bad_packet_count": 0,
            "using_difop_calibration": False,
        },
    )()
    agent._lidar = offline
    agent._handle_lidar_status(Command(action="lidar_status"))
    d = out[0][1]
    assert d["online"] is False
    assert "0 包到达" in d["diagnosis"]


def test_lidar_status_includes_point_count(agent):
    out = []
    agent._send_event_data = lambda e, d: out.append((e, d))
    agent._handle_lidar_status(Command(action="lidar_status"))
    assert out[0][1]["pointCount"] == 0


# ----------------------------------------------------------------------
# point_cloud（点云快照）
# ----------------------------------------------------------------------


def test_point_cloud_sends_snapshot_with_pose(agent):
    out = []
    agent._send_event_data = lambda e, d: out.append((e, d))
    agent._handle_point_cloud(Command(action="point_cloud"))
    assert out
    e, d = out[0]
    assert e == "point_cloud"
    assert d["online"] is True
    assert d["pointCount"] == 4
    assert len(d["points"]) == 4
    assert "pose" in d and "yawDeg" in d["pose"]


def test_point_cloud_offline_when_no_lidar(agent):
    out = []
    agent._send_event_data = lambda e, d: out.append((e, d))
    agent._lidar = None
    agent._handle_point_cloud(Command(action="point_cloud"))
    d = out[0][1]
    assert d["online"] is False
    assert d["points"] == []


def test_point_cloud_command_dispatched(agent):
    """点云命令走 _handle_command 分发表。"""
    out = []
    agent._send_event_data = lambda e, d: out.append((e, d))
    agent._handle_command(Command(action="point_cloud"))
    assert out and out[0][0] == "point_cloud"


# ----------------------------------------------------------------------
# lidar_map + goto 预检
# ----------------------------------------------------------------------


def test_lidar_map_returns_snapshot_and_target_check(agent):
    out = []
    agent._send_event_data = lambda e, d: out.append((e, d))
    # 先注册正前方 2 m 障碍
    agent._occ_grid.update(0.0, 0.0, 0.0, sectors=[(0.0, 2.0)])
    agent._handle_lidar_map(Command(action="lidar_map", x=2.0, y=0.0))
    assert out
    e, d = out[0]
    assert e == "lidar_map"
    assert d["online"] is True
    assert d["targetCheck"]["state"] == "occupied"
    assert "@" in d["text"]


def test_lidar_map_silent_and_include_free_passthrough(agent):
    """云端地图模式轮询：silent 回写进事件数据、includeFree 附带自由格坐标。"""
    out = []
    agent._send_event_data = lambda e, d: out.append((e, d))
    agent._occ_grid.update(0.0, 0.0, 0.0, sectors=[(0.0, 2.0)])
    agent._handle_lidar_map(Command(
        action="lidar_map", silent=True, include_free=True))
    e, d = out[0]
    assert e == "lidar_map"
    assert d.get("silent") is True
    assert d["freeCells"] >= 1
    assert len(d["free"]) == d["freeCells"]
    # 不带 silent 时默认不标记
    agent._handle_lidar_map(Command(action="lidar_map"))
    assert out[1][1].get("silent") is None
    assert out[1][1]["free"] == []


def test_goto_refuses_live_confirmed_blocked_target(agent):
    """伪地图标记占用 + 实时雷达确认目标方位有近障 → goto 拒绝下发。"""
    events = []
    agent._send_event = lambda e, m: events.append((e, m))
    agent._occ_grid.update(0.0, 0.0, 0.0, sectors=[(0.0, 2.0)])
    agent._handle_goto(Command(action="goto", x=2.0, y=0.0))
    assert events and events[-1][0] == "fault"
    assert "实时障碍" in events[-1][1]
    # 导航未下发
    assert agent._navigator.goals == []


def test_goto_proceeds_when_grid_stale_but_live_clear(agent):
    """修复根因：伪地图残留 occupied/blocked（历史瞬时观测烧录），但实时
    雷达目标方位无近障 → goto 应放行，不被陈旧格误拒。"""
    events = []
    agent._send_event = lambda e, m: events.append((e, m))
    # 伪地图烧录了 1.0 m 处的障碍（目标格 occupied），
    # 但实时雷达（_FakeLidar.nearest_in_range → 1.5 m）看不到近于目标的障碍
    agent._occ_grid.update(0.0, 0.0, 0.0, sectors=[(0.0, 1.0)])
    agent._handle_goto(Command(action="goto", x=1.0, y=0.0))
    assert agent._navigator.goals == [(1.0, 0.0, None)]
    assert not events or events[-1][0] != "fault"


def test_goto_free_target_proceeds(agent):
    events = []
    agent._send_event = lambda e, m: events.append((e, m))
    agent._occ_grid.update(0.0, 0.0, 0.0, sectors=[(3.0, 2.0)])
    agent._handle_goto(Command(action="goto", x=1.0, y=0.0))
    assert agent._navigator.goals == [(1.0, 0.0, None)]
    assert not events or events[-1][0] != "fault"


# ----------------------------------------------------------------------
# Navigator drive 回调（导航也走底盘诊断 _drive）
# ----------------------------------------------------------------------


def test_navigator_uses_drive_callback_when_provided():
    ctrl = _Ctrl()
    drive_calls: list = []
    nav = Navigator(
        ctrl, guard=None, wheelbase_m=0.5,
        drive=lambda v, w: drive_calls.append((v, w)),
    )
    nav.goto(0.5, 0.0)
    deadline = time.time() + 3.0
    while nav.is_navigating and time.time() < deadline:
        nav.feed_odometry(100, 100)
        time.sleep(0.01)
    for _ in range(20):
        nav.feed_odometry(100, 100)
        time.sleep(0.005)
    nav.stop()
    assert drive_calls, "drive 回调应被导航线程调用"
    assert not any(c[0] == "ctrl" for c in ctrl.calls), \
        "提供 drive 回调时不应直接走 controller.set_velocity"


def test_navigator_falls_back_to_controller_without_drive():
    ctrl = _Ctrl()
    nav = Navigator(ctrl, guard=None, wheelbase_m=0.5)
    nav.goto(0.5, 0.0)
    deadline = time.time() + 3.0
    while nav.is_navigating and time.time() < deadline:
        nav.feed_odometry(100, 100)
        time.sleep(0.01)
    for _ in range(20):
        nav.feed_odometry(100, 100)
        time.sleep(0.005)
    nav.stop()
    assert any(c[0] == "ctrl" for c in ctrl.calls)


# ----------------------------------------------------------------------
# 风险修复：goto 可选 speed（云端/现场限速） + 雷达掉线 fail-safe
# ----------------------------------------------------------------------


def test_goto_forwards_optional_speed_to_navigator(agent):
    events = []
    agent._send_event = lambda e, m: events.append((e, m))
    agent._occ_grid.update(0.0, 0.0, 0.0, sectors=[(3.0, 2.0)])
    agent._handle_goto(Command(action="goto", x=1.0, y=0.0, speed=0.15))
    assert agent._navigator.goals == [(1.0, 0.0, 0.15)]


def test_goto_zero_speed_uses_default(agent):
    agent._occ_grid.update(0.0, 0.0, 0.0, sectors=[(3.0, 2.0)])
    agent._handle_goto(Command(action="goto", x=1.0, y=0.0, speed=0.0))
    assert agent._navigator.goals == [(1.0, 0.0, None)]


def test_goto_rejects_when_lidar_open_but_no_data(agent):
    """雷达已开启但收不到点云 → goto 立即拒绝（不再「没反应、5s 后 auto_stop」）。"""
    events = []
    agent._send_event = lambda e, m: events.append((e, m))
    offline = type("L", (), {"is_receiving": False})()
    agent._lidar = offline
    agent._handle_goto(Command(action="goto", x=1.0, y=0.0))
    assert events and events[-1][0] == "fault"
    assert "收不到点云" in events[-1][1]
    assert agent._navigator.goals == [], "导航不应下发"


def test_goto_allowed_when_lidar_disabled(agent):
    """lidar off / --no-lidar（_lidar=None）→ 显式无传感器模式，goto 仍可下发。"""
    events = []
    agent._send_event = lambda e, m: events.append((e, m))
    agent._lidar = None
    agent._handle_goto(Command(action="goto", x=1.0, y=0.0))
    assert agent._navigator.goals == [(1.0, 0.0, None)]


def test_navigator_goto_speed_caps_velocity():
    ctrl = _Ctrl()
    drive_calls: list = []
    nav = Navigator(
        ctrl, guard=None, wheelbase_m=0.5,
        drive=lambda v, w: drive_calls.append((v, w)),
    )
    nav.goto(0.5, 0.0, speed=0.06)   # 默认上限 0.30，本次限速 0.06
    deadline = time.time() + 2.0
    while nav.is_navigating and time.time() < deadline:
        nav.feed_odometry(100, 100)
        time.sleep(0.01)
    for _ in range(10):
        nav.feed_odometry(100, 100)
        time.sleep(0.005)
    nav.stop()
    assert drive_calls, "drive 回调应被调用"
    assert all(v <= 0.06 + 1e-6 for v, _ in drive_calls), \
        f"导航线速度应被 speed 上限约束: {drive_calls[:5]}"


def test_goto_speed_too_large_is_rejected():
    ctrl = _Ctrl()
    nav = Navigator(ctrl, guard=None, wheelbase_m=0.5)
    with pytest.raises(ValueError):
        nav.goto(1.0, 0.0, speed=-0.1)


def test_lidar_off_guard_becomes_pass_through(agent):
    """风险1：lidar off 后 guard 换成透传守卫（显式无传感器模式），不锁死运动。"""
    events = []
    agent._send_event = lambda e, m: events.append((e, m))
    agent._handle_lidar_off(Command(action="lidar_off"))
    assert agent._lidar is None
    guard = agent._guard
    assert guard is not None
    assert guard._require_sensor is False
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    assert not blocked and v == pytest.approx(0.3)


def test_lidar_off_guard_wired_into_navigator(agent):
    agent._handle_lidar_off(Command(action="lidar_off"))
    assert getattr(agent._navigator, "guard", None) is agent._guard


# ----------------------------------------------------------------------
# pc_stream 点云实时流
# ----------------------------------------------------------------------


def test_command_hz_parsed_from_payload():
    cmd = Command.from_payload({"action": "pc_stream", "hz": 2.5})
    assert cmd.hz == 2.5
    assert Command.from_payload({"action": "pc_stream"}).hz == 0.0


def test_command_silent_and_include_free_parsed_from_payload():
    cmd = Command.from_payload({
        "action": "lidar_map", "silent": True, "includeFree": True,
    })
    assert cmd.silent is True
    assert cmd.include_free is True
    cmd2 = Command.from_payload({"action": "lidar_map"})
    assert cmd2.silent is False
    assert cmd2.include_free is False


def test_pc_stream_on_off(agent):
    """开流设置频率并上报 info，关流清零。"""
    events = []
    agent._send_event = lambda e, m: events.append((e, m))
    agent._handle_pc_stream(Command(action="pc_stream", hz=4))
    assert agent._pc_stream_hz == 4.0
    assert any(e == "info" and "开启" in m for e, m in events)
    agent._handle_pc_stream(Command(action="pc_stream", hz=0))
    assert agent._pc_stream_hz == 0.0


def test_pc_stream_hz_clamped(agent):
    agent._handle_pc_stream(Command(action="pc_stream", hz=999))
    assert agent._pc_stream_hz == 10.0
    agent._handle_pc_stream(Command(action="pc_stream", hz=-1))
    assert agent._pc_stream_hz == 0.0


def test_pc_stream_dispatched(agent):
    agent._handle_command(Command(action="pc_stream", hz=2))
    assert agent._pc_stream_hz == 2.0


def test_push_point_cloud_stream_throttled(agent):
    """开流后按频率节流推送：同帧内连续调用只发一次，且带 streaming 标记。"""
    out = []
    agent._send_event_data = lambda e, d, log=True: out.append((e, d))
    agent._handle_pc_stream(Command(action="pc_stream", hz=10))
    agent._push_point_cloud_stream()
    agent._push_point_cloud_stream()
    assert len(out) == 1
    e, d = out[0]
    assert e == "point_cloud"
    assert d["online"] is True
    assert d["streaming"] is True


def test_point_cloud_single_shot_not_streaming(agent):
    """单发 point_cloud 命令不带 streaming 标记（云端会保存 PNG）。"""
    out = []
    agent._send_event_data = lambda e, d: out.append((e, d))
    agent._handle_point_cloud(Command(action="point_cloud"))
    assert out[0][0] == "point_cloud"
    assert out[0][1].get("streaming") is None


def test_push_point_cloud_stream_off_when_disabled(agent):
    out = []
    agent._send_event_data = lambda e, d, log=True: out.append((e, d))
    agent._push_point_cloud_stream()
    assert out == []


def test_push_point_cloud_stream_skips_offline(agent):
    """雷达离线时不推点云帧。"""
    out = []
    agent._send_event_data = lambda e, d, log=True: out.append((e, d))
    agent._pc_stream_hz = 5.0
    agent._lidar = type("L", (), {
        "is_receiving": False,
        "point_cloud": lambda **k: {"online": False, "points": []},
    })()
    agent._push_point_cloud_stream()
    assert out == []

