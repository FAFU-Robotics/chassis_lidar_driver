"""Stage-2 rugged-terrain tests: terrain profiling, robust obstacle map,
obstacle-guard terrain behaviours (slope / gravel / multi-frame stop)."""

import math

import pytest

from bunker_mini.lidar import AccumulatingSectors, LidarPoint
from bunker_mini.obstacle import ObstacleGuard, ObstaclePolicy
from bunker_mini.terrain import (
    DEFAULT_MAX_CLIMB_GRADE,
    DEFAULT_STEP_LIMIT_M,
    TerrainProfile,
    TerrainSectorResult,
)


def _pt(az_deg: float, vert_deg: float, dist_m: float, z: float | None = None,
        ch: int = 1) -> LidarPoint:
    if z is None:
        z = dist_m * math.sin(math.radians(vert_deg))
    xy = dist_m * math.cos(math.radians(vert_deg))
    az_r = math.radians(az_deg)
    return LidarPoint(
        azimuth_deg=az_deg,
        vertical_deg=vert_deg,
        distance_m=dist_m,
        reflectivity=100,
        channel=ch,
        x=xy * math.sin(az_r),
        y=xy * math.cos(az_r),
        z=z,
    )


def _frame_with_ground(az_deg: float, ground_z: float,
                       dists: list[float]) -> list[LidarPoint]:
    """Ground points along one azimuth at horizontal distances ``dists``."""
    return [_pt(az_deg, -8.0, d, z=ground_z) for d in dists]


# ---------------------------------------------------------------------------
# TerrainProfile
# ---------------------------------------------------------------------------


def test_flat_ground_is_clear():
    prof = TerrainProfile()
    prof.add_frame(_frame_with_ground(0.0, -0.35, [0.2, 0.3, 0.4, 0.5, 0.6, 0.7]))
    r = prof.sector(0.0, 0.07)
    assert not r.blocked
    assert not r.is_slope
    assert not r.unclear
    assert r.max_height_m < 0.07


def test_step_wall_is_blocked():
    prof = TerrainProfile()
    pts = _frame_with_ground(0.0, -0.35, [0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    # 1.0 m 处出现 0.15 m 高的台阶（> step_limit 0.07）
    pts += _frame_with_ground(0.0, -0.20, [1.0, 1.1, 1.2, 1.3, 1.4, 1.5])
    prof.add_frame(pts)
    r = prof.sector(0.0, 0.07)
    assert r.blocked
    assert r.obstacle_distance_m is not None
    assert 0.9 <= r.obstacle_distance_m <= 1.1
    assert r.max_height_m >= 0.15 - 1e-3


def test_slope_is_not_blocked():
    prof = TerrainProfile()
    pts = []
    for i, d in enumerate([0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2]):
        pts.append(_pt(0.0, -8.0, d, z=-0.35 + 0.3 * d))  # 30% 连续坡
    prof.add_frame(pts)
    r = prof.sector(0.0, 0.07)
    assert r.is_slope
    assert not r.blocked
    assert abs(r.slope_grade - 0.3) < 0.15


def test_gravel_low_bumps_are_passable():
    prof = TerrainProfile()
    pts = _frame_with_ground(0.0, -0.35, [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9])
    # 0.6 m 处一颗 3 cm 碎石：凸起 < step_limit，应可通行
    pts += [_pt(0.0, -8.0, 0.6, z=-0.32)]
    prof.add_frame(pts)
    r = prof.sector(0.0, 0.07)
    assert not r.blocked
    assert r.max_height_m <= 0.05


def test_steep_rock_is_blocked():
    prof = TerrainProfile()
    pts = _frame_with_ground(0.0, -0.35, [0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    # 0.8 m 处 0.4 m 高岩壁（单步突变，不可通行）
    pts += _frame_with_ground(0.0, 0.05, [0.8, 0.9, 1.0, 1.1, 1.2])
    prof.add_frame(pts)
    r = prof.sector(0.0, 0.07)
    assert r.blocked
    assert r.obstacle_distance_m is not None and r.obstacle_distance_m <= 0.9


def test_empty_sector_is_unclear():
    prof = TerrainProfile()
    r = prof.sector(90.0, 0.07)
    assert r.unclear
    assert not r.blocked


def test_nearest_blocked_deg_finds_rock_direction():
    prof = TerrainProfile()
    prof.add_frame(_frame_with_ground(0.0, -0.35, [0.2, 0.3, 0.4, 0.5]))
    # 60° 方向：0.9 m 处仍是地面，1.0 m 处出现 0.4 m 高岩壁前沿
    prof.add_frame(_frame_with_ground(60.0, -0.35, [0.8, 0.9]))
    prof.add_frame(_frame_with_ground(60.0, 0.05, [1.0, 1.1, 1.2]))
    deg = prof.nearest_blocked_deg(0.07)
    assert deg is not None
    assert abs(deg - 60.0) < 6.0


def test_max_climb_grade_turns_steep_slope_blocked():
    prof = TerrainProfile()
    # 45° 坡（grade 1.0）→ 超过 max_climb_grade → 视为墙
    pts = [_pt(0.0, -8.0, d, z=-0.35 + 1.0 * d) for d in [0.3, 0.4, 0.5, 0.6, 0.7]]
    prof.add_frame(pts)
    r = prof.sector(0.0, 0.07, max_climb_grade=0.5)
    assert r.blocked


# ---------------------------------------------------------------------------
# AccumulatingSectors — 低分位抗噪
# ---------------------------------------------------------------------------


def test_accumulating_sectors_quantile_robust():
    acc = AccumulatingSectors(sector_count=120)
    # 真障碍 2.0 m 出现 5 次，碎石毛刺 0.2 m 只出现 1 次
    for _ in range(5):
        acc.add(3.0, 2.0)
    acc.add(3.0, 0.2)
    d = acc.nearest_in_range(0.0, 60.0, quantile=0.25)
    assert d is not None and abs(d - 2.0) < 0.01


def test_accumulating_sectors_picks_true_obstacle_over_spike():
    acc = AccumulatingSectors(sector_count=120)
    for _ in range(3):
        acc.add(180.0, 1.0)
    acc.add(180.0, 0.1)
    d = acc.nearest_in_range(180.0, 30.0, quantile=0.25)
    assert d is not None and d > 0.5


# ---------------------------------------------------------------------------
# ObstacleGuard — 地形行为
# ---------------------------------------------------------------------------


class _FakeLidar:
    """Duck-typed AiryLidar stub — no socket needed."""

    def __init__(self, sectors: AccumulatingSectors | None = None,
                 terrain: TerrainSectorResult | None = None,
                 receiving: bool = True):
        self._acc = sectors or AccumulatingSectors()
        self._terrain = terrain or TerrainSectorResult(angle_deg=0.0)
        self._receiving = receiving

    @property
    def is_receiving(self) -> bool:
        return self._receiving

    def nearest_in_range(self, center_deg: float, width_deg: float,
                         quantile: float = 0.25):
        return self._acc.nearest_in_range(center_deg, width_deg, quantile)

    def terrain_sector(self, angle_deg: float, step_limit_m: float):
        return self._terrain


def _terrain(blocked=False, obstacle_dist=None, slope=False, grade=0.0,
             max_h=0.0, unclear=False, neg_dist=None) -> TerrainSectorResult:
    return TerrainSectorResult(
        angle_deg=0.0,
        obstacle_distance_m=obstacle_dist,
        negative_obstacle_distance_m=neg_dist,
        is_slope=slope,
        slope_grade=grade,
        max_height_m=max_h,
        unclear=unclear,
    )


def test_guard_slope_slows_but_does_not_stop():
    acc = AccumulatingSectors()
    acc.add(0.0, 0.6)  # 坡脚 0.6 m
    lidar = _FakeLidar(acc, _terrain(slope=True, grade=0.3, max_h=0.5))
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    assert not blocked
    assert v < 0.3  # 降速


def test_guard_gravel_slows_but_does_not_stop():
    acc = AccumulatingSectors()
    acc.add(0.0, 0.25)  # 很近的碎石
    lidar = _FakeLidar(acc, _terrain(max_h=0.03))  # 凸起 < step_limit
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    assert not blocked  # 低矮碎石只限速
    assert v < 0.3


def test_guard_real_wall_stops_after_confirmation():
    acc = AccumulatingSectors()
    acc.add(0.0, 0.2)  # 0.2 m 墙
    lidar = _FakeLidar(acc, _terrain(max_h=0.4))  # 真障碍
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=2))
    # 第一帧：确认中，慢速
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    assert not blocked and v < 0.3
    # 第二帧：确认完成 → 急停
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    assert blocked
    assert v == 0.0


def test_guard_terrain_blocked_lookahead_slows():
    acc = AccumulatingSectors()
    acc.add(0.0, 1.5)  # 常规距离远，不触发
    lidar = _FakeLidar(acc, _terrain(blocked=True, obstacle_dist=0.8, max_h=0.2))
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    assert not blocked
    assert v < 0.3  # 提前减速


def test_guard_terrain_blocked_inside_stop_stops():
    acc = AccumulatingSectors()
    acc.add(0.0, 1.5)
    lidar = _FakeLidar(acc, _terrain(blocked=True, obstacle_dist=0.25, max_h=0.2))
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    assert blocked


def test_guard_require_sensor_blocks_when_offline():
    lidar = _FakeLidar(receiving=False)
    guard = ObstacleGuard(lidar, ObstaclePolicy(), require_sensor=True)
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    assert blocked and v == 0.0


def test_guard_sensor_loss_fires_blocked_event():
    """fail-safe：雷达掉线（require_sensor=True）→ 停车并通知云端（obstacle 事件）。"""
    lidar = _FakeLidar(receiving=False)
    guard = ObstacleGuard(lidar, ObstaclePolicy(), require_sensor=True)
    events: list = []
    guard.on_blocked(lambda d, reason: events.append((d, reason)))
    guard.guard_velocity(0.3, 0.0)
    assert len(events) == 1
    assert events[0][0] == 0.0
    assert "雷达掉线" in events[0][1]
    assert "雷达掉线" in guard.last_blocked_reason


def test_guard_sensor_loss_notifies_with_cooldown():
    """掉线通知按 obstacle_event_cooldown_s 去重，不刷屏。"""
    lidar = _FakeLidar(receiving=False)
    guard = ObstacleGuard(lidar, ObstaclePolicy(), require_sensor=True)
    events: list = []
    guard.on_blocked(lambda d, reason: events.append((d, reason)))
    guard.guard_velocity(0.3, 0.0)
    guard.guard_velocity(0.3, 0.0)   # 冷却窗口内：不再推送
    assert len(events) == 1


# ---------------------------------------------------------------------------
# 负障碍（悬崖/坑沿/陨石坑边）检测
# ---------------------------------------------------------------------------


def test_negative_obstacle_dropoff_is_blocked():
    prof = TerrainProfile()
    # 近地地面到 0.5 m，随后出现连续空档（坑沿），1.0 m 后地面重新出现但
    # 下陷 0.2 m（> step_limit 0.07）→ 判为不可通行下坠
    pts = _frame_with_ground(0.0, -0.35, [0.2, 0.3, 0.4, 0.5])
    pts += _frame_with_ground(0.0, -0.55, [1.0, 1.1, 1.2])
    prof.add_frame(pts)
    r = prof.sector(0.0, 0.07)
    assert r.blocked
    assert r.negative_obstacle_distance_m is not None
    assert 0.3 <= r.negative_obstacle_distance_m <= 0.6


def test_negative_obstacle_same_height_gap_is_not_dropoff():
    prof = TerrainProfile()
    # 近地与远处地面同高（-0.35），只是中间空档 → 不是下坠（可能是稀疏）
    pts = _frame_with_ground(0.0, -0.35, [0.2, 0.3, 0.4, 0.5])
    pts += _frame_with_ground(0.0, -0.35, [1.0, 1.1, 1.2])
    prof.add_frame(pts)
    r = prof.sector(0.0, 0.07)
    assert r.negative_obstacle_distance_m is None


def test_guard_negative_obstacle_stops():
    acc = AccumulatingSectors()
    acc.add(0.0, 1.5)  # 常规距离远，不触发
    lidar = _FakeLidar(acc, _terrain(neg_dist=0.3))
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    assert blocked and v == 0.0


def test_guard_speed_adaptive_stop_distance():
    acc = AccumulatingSectors()
    acc.add(0.0, 0.5)  # 0.5 m 真墙
    lidar = _FakeLidar(acc, _terrain(max_h=0.4))
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
    # 低速 0.2 m/s：急停距离 0.37 m < 0.5 → 只限速不急停
    v, w, blocked = guard.guard_velocity(0.2, 0.0)
    assert not blocked and 0.0 < v < 0.2
    # 高速 1.0 m/s：急停距离 0.85 m > 0.5 → 急停（制动距离建模）
    v, w, blocked = guard.guard_velocity(1.0, 0.0)
    assert blocked and v == 0.0


def test_guard_unmeasured_close_hit_is_hard_stop_not_gravel():
    """测高失败（max_h=0）的近处回波必须急停，不能当碎石只限速。"""
    acc = AccumulatingSectors()
    acc.add(0.0, 0.2)
    lidar = _FakeLidar(acc, _terrain(max_h=0.0))
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    assert blocked and v == 0.0


def test_guard_unclear_close_hit_is_hard_stop():
    """地形 unclear 时近处回波按墙处理。"""
    acc = AccumulatingSectors()
    acc.add(0.0, 0.2)
    lidar = _FakeLidar(acc, _terrain(max_h=0.0, unclear=True))
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    assert blocked and v == 0.0


def test_guard_lookahead_includes_measured_wall():
    acc = AccumulatingSectors()
    acc.add(0.0, 0.6)
    lidar = _FakeLidar(acc, _terrain(max_h=0.4))
    guard = ObstacleGuard(lidar, ObstaclePolicy())
    assert guard.front_blocked_lookahead(1.0) == pytest.approx(0.6)


def test_guard_drive_reentry_does_not_undo_hard_stop():
    """nav/patrol 急停后再把 (0,0) 交给 _drive 不得解禁。"""
    acc = AccumulatingSectors()
    acc.add(0.0, 0.2)
    lidar = _FakeLidar(acc, _terrain(max_h=0.4))
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    assert blocked
    v2, w2, blocked2 = guard.guard_velocity(0.0, 0.0)
    assert blocked2 and v2 == 0.0 and w2 == 0.0

