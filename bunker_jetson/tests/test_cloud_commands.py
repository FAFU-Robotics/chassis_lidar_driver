"""云端缺漏补充指令测试：cancel / odom_reset / map_upload / pose_align /
map_return，以及定位健康度、目标结构化上报（mission.targetEst、点云/地图
叠加标记）在状态与快照中的可见性。"""

import json
import time

import pytest

import bunker_mini.agent as agent_mod
from bunker_mini.agent import AgentState, BunkerMiniAgent, Command
from bunker_mini.navigator import Pose2D
from bunker_mini.obstacle import ObstacleGuard, ObstaclePolicy
from bunker_mini.occupancy import OccupancyGrid
from bunker_mini.global_planner import GlobalPlanner
from bunker_mini.terrain import TerrainSectorResult


class _Ctrl:
    def __init__(self) -> None:
        self.calls: list = []
        self.stops = 0
        self.latest_status = None
        self.latest_motion = None
        self.latest_bms = None
        self.latest_odometer = None
        self.live_odometer = None

    def set_velocity(self, v, w):
        self.calls.append(("ctrl", v, w))

    def set_velocity_now(self, v, w):
        self.calls.append(("now", v, w))

    def stop_motion(self):
        self.stops += 1

    def stop(self):
        pass


class _Terrain:
    def all_sectors(self, step_limit=None):
        return [TerrainSectorResult(angle_deg=0.0)]


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


class _Nav:
    def __init__(self) -> None:
        self.pose = Pose2D(3.0, 4.0, 1.0)
        self.is_navigating = False
        self.stopped = 0
        self.reset_calls = 0
        self.goals: list = []

    def stop(self):
        self.stopped += 1
        self.is_navigating = False

    def set_guard(self, guard):
        self.guard = guard

    def reset_pose(self):
        self.reset_calls += 1
        self.pose = Pose2D(0.0, 0.0, 0.0)

    def goto(self, x, y, *, on_arrived=None, on_abort=None, speed=None,
             waypoints=None, replanner=None, goal_yaw=None):
        self.goals.append((x, y, speed))
        return True

    def feed_odometry(self, left_mm, right_mm):
        self.odo = (left_mm, right_mm)

    def apply_external_pose(self, x, y, yaw_deg):
        self.pose = Pose2D(x, y, yaw_deg * 3.141592653589793 / 180.0)

    def apply_yaw_correction(self, yaw_deg):
        self.pose = Pose2D(self.pose.x, self.pose.y, yaw_deg * 3.141592653589793 / 180.0)


class _FakePlayer:
    is_playing = False

    def __init__(self) -> None:
        self.saved: list = []

    def stop(self):
        pass

    def save_track(self, track):
        self.saved.append(track)


class _FakeWs:
    def __init__(self) -> None:
        self.sent: list[str] = []

    def send(self, raw: str) -> None:
        self.sent.append(raw)

    def close(self) -> None:
        pass


@pytest.fixture
def agent(monkeypatch) -> BunkerMiniAgent:
    def _fake_resolve(channel, interface, *, allow_auto_channel=True):
        return "0", "virtual"

    monkeypatch.setattr(agent_mod, "resolve_can_config", _fake_resolve)
    monkeypatch.setattr(agent_mod, "usb_socketcan_channels", lambda: [])
    monkeypatch.setattr(agent_mod, "restore_tx_mode", lambda channels=None: None)
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
    a._planner = GlobalPlanner(a._occ_grid)
    yield a
    a.stop()
    a._cleanup()


def test_local_mode_does_not_need_ws_url(monkeypatch):
    def _fake_resolve(channel, interface, *, allow_auto_channel=True):
        return "0", "virtual"

    monkeypatch.setattr(agent_mod, "resolve_can_config", _fake_resolve)
    a = BunkerMiniAgent(
        ws_url="",
        device_id="TEST-01",
        bind_code="",
        local_mode=True,
    )
    assert a._local_mode
    assert a._ws_url == "local://tcp"
    a._mission_lock = True
    assert a._combat_lock_blocks("move") is False
    a.stop()


# ----------------------------------------------------------------------
# Command.from_payload — 新增云端字段解析
# ----------------------------------------------------------------------


def test_command_from_payload_parses_new_cloud_fields():
    cmd = Command.from_payload({
        "action": "map_upload",
        "cells": [[1.0, 2.0, 3], [0.5, 0.5, 1]],
        "resolution": 0.05,
        "yawDeg": 45.0,
        "mapReturn": True,
    })
    assert cmd.action == "map_upload"
    assert cmd.cells == [[1.0, 2.0, 3], [0.5, 0.5, 1]]
    assert cmd.resolution == 0.05
    assert cmd.yaw_deg == 45.0
    assert cmd.goal_yaw_set is True
    assert cmd.map_return is True
    # 缺失字段安全回退
    cmd2 = Command.from_payload({"action": "cancel"})
    assert cmd2.cells == []
    assert cmd2.resolution == 0.0
    assert cmd2.yaw_deg == 0.0
    assert cmd2.goal_yaw_set is False
    assert cmd2.map_return is False
    assert cmd2.approach is True  # find_object 默认进入对接
    cmd3 = Command.from_payload({"action": "find_object", "approach": False})
    assert cmd3.approach is False


# ----------------------------------------------------------------------
# cancel — 优雅中止并沿轨迹返回
# ----------------------------------------------------------------------


def test_cancel_without_track_stops_and_informs(agent):
    events = []
    agent._send_event = lambda e, m: events.append((e, m))
    agent._navigator.is_navigating = True
    agent._handle_cancel(Command(action="cancel"))
    assert agent._navigator.stopped >= 1, "导航应被停止"
    assert events and events[-1][0] == "find_object"
    assert "直线返回" in events[-1][1] or "无探路轨迹" in events[-1][1]


def test_cancel_with_recording_returns_along_track(agent):
    agent._player = _FakePlayer()
    fake_track = object()
    agent._finalize_recon_track = lambda: fake_track
    returned = []
    agent._return_home = lambda name, track, stop: returned.append((name, track))
    events = []
    agent._send_event = lambda e, m: events.append((e, m))
    agent._handle_cancel(Command(action="cancel", name="rock"))
    time.sleep(0.05)  # 等返回线程跑起来
    assert returned, "cancel 应启动「沿轨迹返回」"
    assert returned[0] == ("rock", fake_track)
    assert any(e == "find_object" for e, _ in events)


# ----------------------------------------------------------------------
# odom_reset — 远程重置里程原点
# ----------------------------------------------------------------------


def test_odom_reset_zeroes_pose_grid_and_mission(agent):
    events = []
    agent._send_event = lambda e, m: events.append((e, m))
    agent._occ_grid.update(1.0, 2.0, 0.0, sectors=[(0.0, 1.5)])
    assert agent._occ_grid.cell_count > 0
    agent._current_target = {"centerX": 1.0, "centerY": 2.0}
    agent._mission = {"status": "recon", "target": "rock"}
    agent._handle_odom_reset(Command(action="odom_reset"))
    assert agent._navigator.reset_calls >= 1
    assert agent._navigator.pose.x == 0.0 and agent._navigator.pose.y == 0.0
    assert agent._occ_grid.cell_count == 0
    assert agent._current_target is None
    assert events and events[-1][0] == "odom_reset"


# ----------------------------------------------------------------------
# 建图通道：map_upload / pose_align / map_return
# ----------------------------------------------------------------------


def test_map_upload_imports_cells_into_grid(agent):
    out = []
    agent._send_event_data = lambda e, d: out.append((e, d))
    agent._handle_map_upload(Command(
        action="map_upload",
        cells=[[1.0, 1.0, 3], [2.0, 2.0, 2], [9.0, 9.0, 5]],
    ))
    assert agent._occ_grid.cell_state(1.0, 1.0) == "blocked"
    assert agent._occ_grid.cell_state(2.0, 2.0) == "occupied"
    assert out and out[0][1]["imported"] == 2


def test_map_upload_rebuilds_grid_on_resolution_change(agent):
    agent._handle_map_upload(Command(
        action="map_upload", cells=[[0.0, 0.0, 2]], resolution=0.05))
    assert agent._occ_grid.resolution_m == pytest.approx(0.05)
    assert agent._occ_grid.cell_state(0.0, 0.0) == "occupied"
    assert agent._planner is not None


def test_map_upload_rejects_empty_cells(agent):
    events = []
    agent._send_event = lambda e, m: events.append((e, m))
    agent._handle_map_upload(Command(action="map_upload", cells=[]))
    assert events and events[-1][0] == "fault"
    assert "cells 为空" in events[-1][1]


def test_pose_align_sets_map_alignment(agent):
    events = []
    agent._send_event = lambda e, m: events.append((e, m))
    agent._handle_pose_align(Command(
        action="pose_align", x=1.5, y=2.5, yaw_deg=30.0))
    assert agent._map_alignment.origin_x == 1.5
    assert agent._map_alignment.origin_y == 2.5
    assert agent._map_alignment.origin_yaw_deg == 30.0
    assert events and events[-1][0] == "pose_align"


def test_map_return_toggles_prefer_map_return(agent):
    events = []
    agent._send_event = lambda e, m: events.append((e, m))
    assert agent._prefer_map_return is False
    agent._handle_map_return(Command(action="map_return", map_return=True))
    assert agent._prefer_map_return is True
    agent._handle_map_return(Command(action="map_return", map_return=False))
    assert agent._prefer_map_return is False


# ----------------------------------------------------------------------
# 定位健康度 + 目标结构化上报可见性
# ----------------------------------------------------------------------


def test_push_state_reports_localization_health(agent):
    agent._loc_source = "scanmatch"
    agent._loc_last_corr = {
        "dx": 0.01, "dy": -0.02, "dyawDeg": 0.3, "at": time.monotonic(),
    }
    agent._state = AgentState.ONLINE
    fake_ws = _FakeWs()
    agent._ws = fake_ws
    agent._push_state()
    assert fake_ws.sent
    payload = json.loads(fake_ws.sent[-1])["payload"]
    assert payload["localization"]["source"] == "scanmatch"
    assert payload["localization"]["wheelbaseM"] == pytest.approx(0.5)
    assert payload["localization"]["lastCorrection"]["dx"] == 0.01
    assert payload["localization"]["lastCorrection"]["agoS"] >= 0.0


def test_push_state_merges_target_est_into_mission(agent):
    agent._current_target = {
        "bearingDeg": 10.0, "distanceM": 2.0, "radiusM": 0.3,
        "centerDistanceM": 2.3, "centerX": 1.0, "centerY": 2.0,
    }
    agent._mission = {"status": "navigating", "target": "rock", "goal": [1.0, 2.0]}
    agent._state = AgentState.ONLINE
    fake_ws = _FakeWs()
    agent._ws = fake_ws
    agent._push_state()
    payload = json.loads(fake_ws.sent[-1])["payload"]
    assert payload["mission"]["targetEst"]["centerX"] == 1.0
    assert payload["mission"]["target"] == "rock"  # 目标名不被覆盖


def test_point_cloud_snapshot_includes_target_marker(agent):
    agent._current_target = {"centerX": 1.0, "centerY": 2.0}
    data = agent._collect_point_cloud_data()
    assert data["target"]["centerX"] == 1.0
    agent._current_target = None
    data = agent._collect_point_cloud_data()
    assert "target" not in data


def test_lidar_map_includes_target_marker(agent):
    out = []
    agent._send_event_data = lambda e, d: out.append((e, d))
    agent._current_target = {"centerX": 1.0, "centerY": 2.0}
    agent._handle_lidar_map(Command(action="lidar_map"))
    assert out and out[0][1]["target"]["centerX"] == 1.0


# ----------------------------------------------------------------------
# 指令分发路由
# ----------------------------------------------------------------------


def test_move_zero_stops_immediately(agent):
    """move(0,0) 必须 stop_motion，不能斜坡残速被 TX 保活。"""
    agent._handle_command(Command(action="move", v=0.1, w=0.0, bypass_guard=True))
    agent._handle_command(Command(action="move", v=0.0, w=0.0, bypass_guard=True))
    assert agent._controller.stops >= 1
    assert agent._pending_command is None


def test_kb_move_bypasses_accel_ramp(agent):
    """kb move 必须 set_velocity_now，起步不走 0.5 m/s² 斜坡。"""
    agent._handle_command(Command(action="move", v=0.1, w=0.0, bypass_guard=True))
    assert agent._controller.calls[-1] == ("now", 0.1, 0.0)
    agent._handle_command(Command(action="move", v=0.1, w=0.0, bypass_guard=False))
    assert agent._controller.calls[-1] == ("ctrl", 0.1, 0.0)


def test_kb_move_caps_duration(agent):
    """kb 单包 duration 必须封顶，停车包丢失时不能再冲近 1 秒。"""
    from bunker_mini.agent import TELEOP_DURATION_CAP_S

    now = time.time()
    agent._handle_command(
        Command(action="move", v=0.1, w=0.0, duration=0.90, bypass_guard=True)
    )
    remain = agent._move_deadline - now
    assert remain <= TELEOP_DURATION_CAP_S + 0.05


def test_handle_command_routes_new_actions(agent, monkeypatch):
    routed = []
    for name in ("cancel", "odom_reset", "map_upload", "pose_align", "map_return"):
        monkeypatch.setattr(
            agent, f"_handle_{name}", lambda cmd, n=name: routed.append(n))
    for name in ("cancel", "odom_reset", "map_upload", "pose_align", "map_return"):
        agent._handle_command(Command(action=name))
    assert routed == ["cancel", "odom_reset", "map_upload", "pose_align", "map_return"]


# ----------------------------------------------------------------------
# 控制模式 / 车辆状态上报（遥控器抢占监控）
# ----------------------------------------------------------------------


def _fake_status(**kw):
    return type("S", (), {
        "fault_code": 0,
        "control_mode": 3,
        "vehicle_state": 1,
        **kw,
    })()


def test_push_state_reports_control_mode_and_vehicle_state(agent):
    agent._controller.latest_status = _fake_status()
    agent._state = AgentState.ONLINE
    fake_ws = _FakeWs()
    agent._ws = fake_ws
    agent._push_state()
    payload = json.loads(fake_ws.sent[-1])["payload"]
    assert payload["modeCode"] == 3
    assert "mode" in payload and payload["mode"]
    assert payload["vehicleStateCode"] == 1
    assert "vehicleState" in payload and payload["vehicleState"]


def test_push_state_force_emits_tcp_during_stick(agent):
    sent: list[dict] = []

    class _Tcp:
        clients = 1

        def stick_age_s(self):
            return 0.01

        def push_event(self, msg):
            sent.append(msg)

    agent._local_mode = True
    agent._teleop_tcp = _Tcp()
    agent._controller.latest_status = _fake_status(
        control_mode=1, vehicle_state=0, battery_voltage_v=26.0, count=1,
    )
    agent._state = AgentState.ONLINE
    agent._ws = None
    agent._push_state()
    assert sent and sent[-1]["type"] == "state"
    sent.clear()
    agent._push_state()
    assert sent == []
    agent._push_state(force=True)
    assert sent and sent[-1]["payload"]["chassis"]["heard"] is True


def test_push_state_includes_chassis_manual_fields(agent):
    agent._controller.latest_status = _fake_status(
        fault_code=0x02,
        control_mode=1,
        vehicle_state=0,
        battery_voltage_v=26.4,
        count=17,
    )
    agent._controller.latest_bms = type("B", (), {
        "soc_percent": 78,
        "soh_percent": 96,
        "voltage_v": 26.35,
        "current_a": -1.2,
        "temperature_c": 31.5,
    })()
    agent._controller.latest_motion = type("M", (), {
        "linear_velocity_m_s": 0.12,
        "angular_velocity_rad_s": -0.05,
    })()
    agent._controller.latest_odometer = type("O", (), {
        "left_wheel_mm": 1234,
        "right_wheel_mm": 1200,
    })()
    agent._state = AgentState.ONLINE
    fake_ws = _FakeWs()
    agent._ws = fake_ws
    agent._push_state()
    chassis = json.loads(fake_ws.sent[-1])["payload"]["chassis"]
    assert chassis["heard"] is True
    assert chassis["system"]["batteryVoltageV"] == 26.4
    assert chassis["system"]["faultHex"] == "0x02"
    assert "电池欠压警告(15%)" in chassis["system"]["faults"]
    assert chassis["bms"]["socPercent"] == 78
    assert chassis["bms"]["currentA"] == -1.2
    assert chassis["motion"]["linearMs"] == 0.12
    assert chassis["odometer"]["leftMm"] == 1234


def test_push_state_omits_mode_when_status_unavailable(agent):
    agent._controller.latest_status = None
    agent._state = AgentState.ONLINE
    fake_ws = _FakeWs()
    agent._ws = fake_ws
    agent._push_state()
    payload = json.loads(fake_ws.sent[-1])["payload"]
    assert "mode" not in payload
    assert "vehicleState" not in payload


# ----------------------------------------------------------------------
# CAN 通道周期重探：底盘稍后才在 can1 上线时，agent 应热切换
# ----------------------------------------------------------------------


class _SwappableCtrl(_Ctrl):
    def __init__(self) -> None:
        super().__init__()
        self.switched_to: list = []

    def switch_channel(self, channel, interface=None) -> None:
        self.switched_to.append((channel, interface))


def test_reprobe_skipped_when_channel_explicit(agent, monkeypatch) -> None:
    """用户显式指定通道 → 不做自动重探（尊重用户选择）。"""
    agent._can_channel_explicit = True
    agent._can_channel = "can0"
    calls = []

    def _fake_probe(*_a, **_k):
        calls.append(True)
        return "can1"

    monkeypatch.setattr(agent_mod, "probe_chassis_channel", _fake_probe)
    ctrl = _SwappableCtrl()
    agent._controller = ctrl
    agent._maybe_reprobe_can_channel(time.time())
    assert calls == []
    assert ctrl.switched_to == []


def test_reprobe_skipped_on_interval(agent, monkeypatch) -> None:
    """间隔未到 → 不重复探测。"""
    agent._can_channel_explicit = False
    agent._can_channel = "can0"
    agent._last_can_reprobe_at = time.time()  # 刚探测过

    calls = []

    def _fake_probe(*_a, **_k):
        calls.append(True)
        return "can1"

    monkeypatch.setattr(agent_mod, "probe_chassis_channel", _fake_probe)
    ctrl = _SwappableCtrl()
    agent._controller = ctrl
    agent._maybe_reprobe_can_channel(time.time())
    assert calls == []
    assert ctrl.switched_to == []


def test_reprobe_switches_to_late_channel(agent, monkeypatch) -> None:
    """启动后底盘出现在新通道 can1 → 热切换控制器与 _can_channel。"""
    agent._can_channel_explicit = False
    agent._can_channel = "can0"
    agent._last_can_reprobe_at = 0.0

    monkeypatch.setattr(
        agent_mod, "probe_chassis_channel",
        lambda *_a, **_k: "can1",
    )
    ctrl = _SwappableCtrl()
    agent._controller = ctrl
    agent._maybe_reprobe_can_channel(time.time())
    assert ctrl.switched_to == [("can1", agent._can_interface)]
    assert agent._can_channel == "can1"


def test_reprobe_keeps_channel_when_nothing_found(agent, monkeypatch) -> None:
    """重探没找到新通道 → 保持原通道不变。"""
    agent._can_channel_explicit = False
    agent._can_channel = "can0"
    agent._last_can_reprobe_at = 0.0

    monkeypatch.setattr(
        agent_mod, "probe_chassis_channel",
        lambda *_a, **_k: None,
    )
    ctrl = _SwappableCtrl()
    agent._controller = ctrl
    agent._maybe_reprobe_can_channel(time.time())
    assert ctrl.switched_to == []
    assert agent._can_channel == "can0"


def test_reprobe_falls_back_to_usb_when_silent(agent, monkeypatch) -> None:
    """重探仍无 0x211 → 切到 USB-CAN（按 sysfs，不按接口名）。"""
    agent._can_channel_explicit = False
    agent._can_channel = "can0"
    agent._last_can_reprobe_at = 0.0
    monkeypatch.setattr(
        agent_mod, "probe_chassis_channel",
        lambda *_a, **_k: None,
    )
    monkeypatch.setattr(agent_mod, "usb_socketcan_channels", lambda: ["can1"])
    ctrl = _SwappableCtrl()
    agent._controller = ctrl
    agent._maybe_reprobe_can_channel(time.time())
    assert ctrl.switched_to == [("can1", agent._can_interface)]
    assert agent._can_channel == "can1"


def test_idle_stick_does_not_kill_timed_move(agent) -> None:
    """网页 100Hz 空闲 stick(0,0) 不得清掉定时 move。"""
    cmd = Command(action="move", v=0.2, w=0.0, duration=3.0)
    agent._pending_command = cmd
    agent._move_deadline = time.time() + 3.0
    agent._teleop_stick(0.0, 0.0)
    assert agent._pending_command is cmd
    assert agent._controller.stops == 0


def test_idle_stick_does_not_kill_goto(agent) -> None:
    agent._navigator.is_navigating = True
    agent._teleop_stick(0.0, 0.0)
    assert agent._controller.stops == 0
    assert agent._navigator.stopped == 0


def test_single_zero_stick_does_not_stop_hid_hold(agent) -> None:
    """TCP HID 长按中偶发一帧 (0,0) 不得立刻停车。"""
    agent._teleop_stick(0.0, 0.40)
    assert agent._controller.stops == 0
    agent._teleop_stick(0.0, 0.0)
    assert agent._controller.stops == 0
    agent._teleop_stick(0.0, 0.40)
    assert agent._controller.stops == 0


def test_two_zero_sticks_mean_real_release(agent) -> None:
    agent._teleop_stick(0.0, 0.40)
    agent._teleop_stick(0.0, 0.0)
    agent._teleop_stick(0.0, 0.0)
    assert agent._controller.stops >= 1


def test_nonzero_stick_takes_over_goto(agent) -> None:
    agent._navigator.is_navigating = True
    agent._teleop_stick(0.12, 0.0)
    assert agent._navigator.stopped == 1
    assert agent._pending_command is None


def test_idle_watchdog_spares_timed_move(agent) -> None:
    agent._pending_command = Command(action="move", v=0.2, w=0.0, duration=3.0)
    agent._move_deadline = time.time() + 3.0
    agent._teleop_idle_stop()
    assert agent._controller.stops == 0


def test_move_rejected_when_lidar_offline(agent) -> None:
    agent._state = AgentState.ONLINE
    agent._lidar.is_receiving = False
    agent._handle_command(Command(action="move", v=0.2, w=0.0, duration=2.0))
    assert agent._pending_command is None
    assert agent._controller.stops == 0


def test_open_loop_move_runs_when_lidar_offline(agent) -> None:
    agent._state = AgentState.ONLINE
    agent._lidar.is_receiving = False
    agent._handle_command(
        Command(action="move", v=0.2, w=0.0, duration=2.0, open_loop=True)
    )
    assert agent._pending_command is not None
    assert agent._pending_command.action == "move"
    assert agent._timed_move_active()


def test_guarded_move_keeps_commanded_speed(agent) -> None:
    """守卫把本拍打成 0 时，pending 仍保留原速，后续拍才能恢复。"""

    class _Block:
        def guard_velocity(self, v, w):
            return 0.0, 0.0, True

    agent._state = AgentState.ONLINE
    agent._lidar.is_receiving = True
    agent._guard = _Block()
    agent._handle_command(Command(action="move", v=0.15, w=0.0, duration=2.0))
    assert agent._pending_command is not None
    assert agent._pending_command.v == 0.15
    assert agent._pending_command.w == 0.0
    assert agent._timed_move_active()


def test_open_loop_refresh_does_not_reapply_guard(agent) -> None:
    """网页开环 move 不得被 0.2s 看门狗再套守卫打成 0。"""

    class _Block:
        def guard_velocity(self, v, w):
            return 0.0, 0.0, True

    agent._state = AgentState.ONLINE
    agent._guard = _Block()
    agent._handle_command(
        Command(action="move", v=0.15, w=0.0, duration=2.0, open_loop=True)
    )
    stops = agent._controller.stops
    calls = list(agent._controller.calls)
    agent._refresh_guarded_move()
    assert agent._controller.stops == stops
    assert agent._controller.calls == calls
    assert calls and calls[-1][1] == 0.15


def test_reprobe_same_channel_does_not_rebuild(agent, monkeypatch) -> None:
    class _RebuildCtrl(_SwappableCtrl):
        def __init__(self) -> None:
            super().__init__()
            self.rebuilds = 0

        def rebuild_bus(self) -> None:
            self.rebuilds += 1

    agent._can_channel_explicit = False
    agent._can_channel = "can0"
    agent._last_can_reprobe_at = 0.0
    monkeypatch.setattr(
        agent_mod, "probe_chassis_channel",
        lambda *_a, **_k: "can0",
    )
    ctrl = _RebuildCtrl()
    agent._controller = ctrl
    agent._maybe_reprobe_can_channel(time.time())
    assert ctrl.rebuilds == 0
    assert ctrl.switched_to == []
    assert agent._can_channel == "can0"


def test_sync_nav_odometry_prefers_live(agent):
    agent._controller.live_odometer = type("O", (), {
        "left_wheel_mm": 1200, "right_wheel_mm": 1180,
    })()
    agent._sync_nav_odometry()
    assert agent._navigator.odo == (1200, 1180)


def test_scan_match_skip_only_fast_spin(agent):
    """慢速 goto 转向（<12°/拍）必须仍做匹配，不能整拍跳过。"""
    import math
    from bunker_mini.scanmatch import ScanMatcher

    seen = []

    class _Matcher(ScanMatcher):
        def match(self, sectors, x, y, yaw_deg):
            seen.append(yaw_deg)
            return None

        def observe(self, sectors, x, y, yaw_deg):
            return None

    agent._scan_matcher = _Matcher()
    agent._scan_match_last_yaw = 0.0
    agent._navigator.pose = Pose2D(0.1, 0.0, math.radians(8.0))
    agent._scan_match_tick()
    assert seen, "8° 转向应进入 match"
    seen.clear()
    agent._navigator.pose = Pose2D(0.1, 0.0, math.radians(8.0 + 20.0))
    agent._scan_match_tick()
    assert not seen, "20° 猛转才跳过"


def test_scan_match_yaw_only_keeps_xy(agent):
    from bunker_mini.scanmatch import ScanMatcher

    class _Matcher(ScanMatcher):
        def match(self, sectors, x, y, yaw_deg):
            return (0.50, -0.40, 6.0)

        def observe(self, sectors, x, y, yaw_deg):
            return None

    agent._scan_matcher = _Matcher()
    agent._scan_match_blend = 1.0
    agent._scan_match_last_yaw = None
    agent._navigator.pose = Pose2D(2.0, 3.0, 0.0)
    agent._scan_match_tick(yaw_only=True)
    assert agent._navigator.pose.x == pytest.approx(2.0)
    assert agent._navigator.pose.y == pytest.approx(3.0)
    assert agent._navigator.pose.yaw_deg == pytest.approx(6.0, abs=0.05)
    assert agent._loc_source == "yawmatch"
