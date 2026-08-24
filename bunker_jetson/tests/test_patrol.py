"""Tests for the autonomous recon patrol controller (自主探路)."""

import math

import pytest

from bunker_mini.patrol import PatrolConfig, PatrolController, PatrolState
from bunker_mini.terrain import TerrainSectorResult


class _FakeLidar:
    """Direction→distance stand-in: exact per-heading distance, optional
    terrain-blocked set. Probes the given heading directly."""

    def __init__(self, distances: dict[float, float | None],
                 terrain_blocked: set[float] | None = None) -> None:
        # normalize keys to [0, 360) to mirror the real AccumulatingSectors
        self._distances = {(k % 360): v for k, v in distances.items()}
        self._terrain_blocked = {(k % 360) for k in (terrain_blocked or set())}
        self.is_receiving = True

    def nearest_in_range(self, center_deg: float, _width: float) -> float | None:
        return self._distances.get(center_deg % 360)

    def terrain_sector(self, angle_deg: float) -> TerrainSectorResult:
        blocked = angle_deg % 360 in self._terrain_blocked
        return TerrainSectorResult(
            angle_deg=angle_deg,
            obstacle_distance_m=0.5 if blocked else None,
            max_height_m=0.0, slope_grade=0.0)


class _FakeController:
    def __init__(self) -> None:
        self.last_v = 0.0
        self.last_w = 0.0
        self.stopped = False

    def set_velocity(self, v: float, w: float) -> None:
        self.last_v = v
        self.last_w = w

    def stop_motion(self) -> None:
        self.stopped = True
        self.last_v = 0.0
        self.last_w = 0.0


def _open_field_cfg(**kw) -> PatrolConfig:
    """开阔方向盲探的测试配置（关闭贴壁跟随，保留原语义）。"""
    return PatrolConfig(wall_follow_enabled=False, **kw)


def test_patrol_heads_straight_when_clear() -> None:
    # 前方 2m、左右也开阔 → 选择 0° 直行
    lidar = _FakeLidar({0: 2.0, 30: 1.5, -30: 1.5, 60: 1.0, -60: 1.0})
    ctrl = _FakeController()
    patrol = PatrolController(ctrl, None, lidar, config=_open_field_cfg())
    patrol.start()
    v, w = patrol.update()
    assert v > 0.0
    assert abs(w) < 1e-9  # 正前方 → 不转
    assert patrol.state == PatrolState.MOVING
    assert patrol.heading_deg == 0.0


def test_patrol_turns_to_open_side_when_front_blocked() -> None:
    # 正前方 0.2m 被堵（< min_clear），左前方 30° 开阔 → 选择 30° 左转
    lidar = _FakeLidar({0: 0.2, 30: 2.0, -30: 0.3, 60: 1.0, -60: 1.0,
                        90: 1.0, -90: 1.0, 120: 1.0, -120: 1.0,
                        150: 1.0, -150: 1.0})
    ctrl = _FakeController()
    patrol = PatrolController(ctrl, None, lidar, config=_open_field_cfg())
    patrol.start()
    v, w = patrol.update()
    assert patrol.heading_deg == 30.0
    assert v > 0.0
    assert w > 0.0  # 左转


def test_patrol_avoids_terrain_blocked_heading() -> None:
    # 30° 方向地形不可通行（台阶/岩壁），虽开阔也要跳过
    lidar = _FakeLidar(
        {0: 0.2, 30: 3.0, -30: 2.0, 60: 1.0, -60: 1.0,
         90: 1.0, -90: 1.0, 120: 1.0, -120: 1.0, 150: 1.0, -150: 1.0},
        terrain_blocked={30},
    )
    ctrl = _FakeController()
    patrol = PatrolController(ctrl, None, lidar, config=_open_field_cfg())
    patrol.start()
    v, w = patrol.update()
    assert patrol.heading_deg == -30.0  # 跳过 30°，选 -30°
    assert v > 0.0
    assert w < 0.0  # 右转


def test_patrol_turns_in_place_when_everything_blocked() -> None:
    # 所有方向都被堵 → 原地转向（w>0），v=0
    lidar = _FakeLidar({0: 0.2, 30: 0.2, -30: 0.2, 60: 0.2, -60: 0.2,
                        90: 0.2, -90: 0.2, 120: 0.2, -120: 0.2,
                        150: 0.2, -150: 0.2})
    ctrl = _FakeController()
    patrol = PatrolController(ctrl, None, lidar, config=_open_field_cfg())
    patrol.start()
    v, w = patrol.update()
    assert v == 0.0
    assert w > 0.0
    assert patrol.state in (PatrolState.TURNING,)


def test_patrol_backs_up_after_turn_timeout() -> None:
    cfg = _open_field_cfg(turn_timeout_s=0.05, backup_duration_s=0.1)
    lidar = _FakeLidar({0: 0.2, 30: 0.2, -30: 0.2, 60: 0.2, -60: 0.2,
                        90: 0.2, -90: 0.2, 120: 0.2, -120: 0.2,
                        150: 0.2, -150: 0.2})
    ctrl = _FakeController()
    patrol = PatrolController(ctrl, None, lidar, config=cfg)
    patrol.start()
    import time
    for _ in range(3):
        v, w = patrol.update()  # TURNING
        assert v == 0.0 and w > 0.0
    time.sleep(0.06)
    v, w = patrol.update()  # 超过 turn_timeout → 倒车
    assert v < 0.0
    assert w == 0.0
    assert patrol.state == PatrolState.BACKING


def test_patrol_slows_near_front_obstacle() -> None:
    # 前方较近（0.5m）→ 低于全速
    lidar = _FakeLidar({0: 0.5, 30: 1.5, -30: 1.5})
    ctrl = _FakeController()
    patrol = PatrolController(ctrl, None, lidar, config=_open_field_cfg())
    patrol.start()
    v, _ = patrol.update()
    cfg = PatrolConfig()
    assert 0.0 < v < cfg.max_linear_m_s


def test_patrol_stopped_returns_zero() -> None:
    lidar = _FakeLidar({0: 2.0, 30: 1.5, -30: 1.5})
    ctrl = _FakeController()
    patrol = PatrolController(ctrl, None, lidar)
    patrol.start()
    patrol.stop()
    v, w = patrol.update()
    assert v == 0.0 and w == 0.0
    assert ctrl.stopped


def test_patrol_no_lidar_keeps_moving_forward() -> None:
    # 雷达离线（nearest_in_range → None）→ 无法判断，直行交给 guard 兜底
    lidar = _FakeLidar({})
    ctrl = _FakeController()
    patrol = PatrolController(ctrl, None, lidar)
    patrol.start()
    v, w = patrol.update()
    assert v > 0.0


# ---------------------------------------------------------------------------
# 贴壁跟随（溶洞走廊系统性覆盖）
# ---------------------------------------------------------------------------


def test_patrol_wall_follow_steers_toward_distant_wall() -> None:
    # 左侧壁在 0.9m（在贴壁范围内但比期望偏移 0.6m 远）→ 向壁靠近（左转）
    lidar = _FakeLidar({0: 3.0, 30: 0.9, 60: 1.0, 90: 1.2})
    ctrl = _FakeController()
    patrol = PatrolController(ctrl, None, lidar)
    patrol.start()
    v, w = patrol.update()
    assert patrol.state == PatrolState.MOVING
    assert patrol.heading_deg > 0.0  # 左转靠近壁
    assert w > 0.0


def test_patrol_wall_follow_steers_away_from_near_wall() -> None:
    # 左侧壁只有 0.3m（比期望偏移近）→ 向右避开
    lidar = _FakeLidar({0: 3.0, 30: 0.3, 60: 0.5, 90: 0.8})
    ctrl = _FakeController()
    patrol = PatrolController(ctrl, None, lidar)
    patrol.start()
    v, w = patrol.update()
    assert patrol.heading_deg < 0.0  # 右转远离壁
    assert w < 0.0


def test_patrol_wall_follow_ignored_without_near_wall() -> None:
    # 侧向无近壁（全部 > wall_follow_range）→ 回退开阔方向盲探，直行
    lidar = _FakeLidar({0: 3.0, 30: 3.0, 60: 3.0, 90: 3.0,
                        -30: 3.0, -60: 3.0, -90: 3.0})
    ctrl = _FakeController()
    patrol = PatrolController(ctrl, None, lidar)
    patrol.start()
    v, w = patrol.update()
    assert patrol.heading_deg == 0.0
    assert v > 0.0


def test_patrol_wall_follow_disabled_by_config() -> None:
    cfg = PatrolConfig(wall_follow_enabled=False)
    lidar = _FakeLidar({0: 3.0, 30: 0.9, 60: 1.0, 90: 1.2})
    ctrl = _FakeController()
    patrol = PatrolController(ctrl, None, lidar, config=cfg)
    patrol.start()
    v, w = patrol.update()
    assert patrol.heading_deg == 0.0  # 关闭贴壁 → 纯开阔选择


# ---------------------------------------------------------------------------
# 死胡同 U 形调头 & 正后方采样
# ---------------------------------------------------------------------------


def test_patrol_dead_end_forced_u_turn_after_stall() -> None:
    # 全部方向被堵（含正后方）→ 转向超时先倒车 → 倒完仍无路强制 180° 调头
    import time
    cfg = _open_field_cfg(turn_timeout_s=0.05, backup_duration_s=0.08,
                          u_turn_duration_s=0.5)
    lidar = _FakeLidar({0: 0.2, 30: 0.2, -30: 0.2, 60: 0.2, -60: 0.2,
                        90: 0.2, -90: 0.2, 120: 0.2, -120: 0.2,
                        150: 0.2, -150: 0.2, 165: 0.2, -165: 0.2,
                        180: 0.2, -180: 0.2})
    ctrl = _FakeController()
    patrol = PatrolController(ctrl, None, lidar, config=cfg)
    patrol.start()
    for _ in range(3):
        patrol.update()  # 初始 TURNING
    time.sleep(0.06)
    v, w = patrol.update()  # 超过 turn_timeout → 先倒车脱困
    assert v < 0.0 and w == 0.0
    assert patrol.state == PatrolState.BACKING
    time.sleep(0.09)  # 倒车结束仍无路 → 进入强制调头
    v, w = patrol.update()
    assert v == 0.0 and w > 0.0
    assert patrol.state == PatrolState.TURNING
    # 调头期间持续原地旋转（不打 STUCK）
    time.sleep(0.2)
    v, w = patrol.update()
    assert v == 0.0 and w > 0.0


def test_patrol_turns_around_when_only_rear_open() -> None:
    # 前方/左右全堵，仅正后方开阔 → 转身（±165°~180°）探索来路分支
    lidar = _FakeLidar({0: 0.2, 30: 0.2, -30: 0.2, 60: 0.2, -60: 0.2,
                        90: 0.2, -90: 0.2, 120: 0.2, -120: 0.2,
                        150: 0.2, -150: 0.2, 180: 2.0, -180: 2.0})
    ctrl = _FakeController()
    patrol = PatrolController(ctrl, None, lidar, config=_open_field_cfg())
    patrol.start()
    v, w = patrol.update()
    assert abs(patrol.heading_deg) >= 165.0
    assert v > 0.0
