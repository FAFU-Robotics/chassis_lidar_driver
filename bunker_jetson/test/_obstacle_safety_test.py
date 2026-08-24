#!/usr/bin/env python3
"""避障安全网回归：绕过守卫的通路、测高失败当墙、急停必须直通停车。"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from bunker_mini.lidar import AccumulatingSectors, AiryLidar
from bunker_mini.navigator import NavigateConfig, Navigator
from bunker_mini.obstacle import (
    ObstacleGuard,
    ObstaclePolicy,
    near_collision_hits,
)
from bunker_mini.terrain import TerrainSectorResult


class _P:
    def __init__(self, x, y, z) -> None:
        self.x, self.y, self.z = x, y, z


class _FakeLidar:
    def __init__(self, front: float, terrain: TerrainSectorResult | None = None,
                 receiving: bool = True, body_points=None) -> None:
        self._acc = AccumulatingSectors()
        self._acc.add(0.0, front)
        self._acc.add(180.0, 5.0)
        self._terrain = terrain or TerrainSectorResult(angle_deg=0.0)
        self.is_receiving = receiving
        self.latest_frame = type("F", (), {"points": list(body_points or [])})()

    def nearest_in_range(self, center_deg, width_deg, quantile=0.25):
        return self._acc.nearest_in_range(center_deg, width_deg, quantile)

    def terrain_sector(self, angle_deg, step_limit_m=0.07):
        return self._terrain


class _FakeCtrl:
    def __init__(self) -> None:
        self.vel: list[tuple[float, float]] = []
        self.stops = 0

    def set_velocity(self, v, w) -> None:
        self.vel.append((v, w))

    def stop_motion(self) -> None:
        self.stops += 1


def check(name: str, cond: bool, detail: str = "") -> None:
    if cond:
        print(f"  PASS  {name}")
        return
    print(f"  FAIL  {name}  {detail}")
    raise SystemExit(1)


def test_unmeasured_wall_stops() -> None:
    lidar = _FakeLidar(0.2, TerrainSectorResult(angle_deg=0.0, max_height_m=0.0))
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    check("测高失败的近处回波急停", blocked and v == 0.0)


def test_gravel_still_slows() -> None:
    lidar = _FakeLidar(0.25, TerrainSectorResult(angle_deg=0.0, max_height_m=0.03))
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    check("明确矮碎石不急停", not blocked)
    check("明确矮碎石限速", 0.0 < v < 0.3)


def test_stale_timeout_is_tight() -> None:
    check("直播雷达掉线判定 ≤0.5s", AiryLidar.NO_DATA_STALE_S <= 0.5)
    check("直播雷达掉线判定 ≥0.2s", AiryLidar.NO_DATA_STALE_S >= 0.2)


def test_zero_move_does_not_cry_lidar_lost() -> None:
    """kb 进门会发 move 0 0：雷达还没出点时不得刷「掉线急停」。"""
    lidar = _FakeLidar(5.0, receiving=False)
    events: list = []
    guard = ObstacleGuard(lidar, ObstaclePolicy(), require_sensor=True)
    guard.on_blocked(lambda d, reason: events.append(reason))
    v, w, blocked = guard.guard_velocity(0.0, 0.0)
    check("停车不报掉线", not blocked and v == 0.0 and not events)
    v, w, blocked = guard.guard_velocity(0.10, 0.0)
    check("加油门才拦掉线", blocked and v == 0.0 and events)


def test_navigator_backup_is_guarded() -> None:
    """无 _drive 时倒车也必须过守卫：正后方有墙则 stop_motion，不得 set_velocity 倒进去。"""
    lidar = _FakeLidar(5.0)
    lidar._acc = AccumulatingSectors()
    lidar._acc.add(0.0, 5.0)
    lidar._acc.add(180.0, 0.15)
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1),
                          require_sensor=True)
    ctrl = _FakeCtrl()
    nav = Navigator(ctrl, guard, wheelbase_m=0.5,
                    config=NavigateConfig(update_interval_s=0.05))
    nav._set_vel(-0.12, 0.0)
    check("倒向近处障碍时 stop_motion", ctrl.stops >= 1)
    check("倒向近处障碍时不得下发负速度", not any(v < 0 for v, _w in ctrl.vel))


def test_body_box_stops_chair_when_sector_is_empty() -> None:
    """椅背只被仰视激光打到：水平扇区是空的，立体盒必须急停。"""
    lidar = _FakeLidar(5.0, body_points=[_P(0.05, 0.28, 0.55)])
    lidar._acc = AccumulatingSectors()
    lidar._acc.add(0.0, 5.0)
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
    v, w, blocked = guard.guard_velocity(0.10, 0.0)
    check("椅背立体盒急停", blocked and v == 0.0)


def test_chair_at_half_meter_hard_stops() -> None:
    """现场椅背约 0.46 m：低速普通 stop 只有 0.36 m，必须按人体/椅背急停。"""
    pts = [_P(-0.08 + 0.04 * i, 0.46, 0.48 + 0.02 * (i % 2)) for i in range(5)]
    lidar = _FakeLidar(5.0, body_points=pts)
    lidar._acc = AccumulatingSectors()
    lidar._acc.add(0.0, 5.0)
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
    v, w, blocked = guard.guard_velocity(0.10, 0.0)
    check("0.46 m 椅背急停", blocked and v == 0.0)


def test_body_box_hold_survives_one_empty_frame() -> None:
    """椅腿点云闪空一帧不得放行 0.10 m/s。"""
    pts = [_P(-0.06 + 0.03 * i, 0.34, 0.50) for i in range(5)]
    lidar = _FakeLidar(5.0, body_points=pts)
    lidar._acc = AccumulatingSectors()
    lidar._acc.add(0.0, 5.0)
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
    guard.guard_velocity(0.10, 0.0)
    lidar.latest_frame.points = []
    v, w, blocked = guard.guard_velocity(0.10, 0.0)
    check("丢点后仍保持近距", blocked or v < 0.05)


def test_lookahead_uses_body_box() -> None:
    """goto 提前绕行必须看见椅背，不能等急停才转向。"""
    pts = [_P(-0.08 + 0.04 * i, 0.55, 0.50) for i in range(5)]
    lidar = _FakeLidar(5.0, body_points=pts)
    lidar._acc = AccumulatingSectors()
    lidar._acc.add(0.0, 5.0)
    guard = ObstacleGuard(lidar, ObstaclePolicy())
    d = guard.front_blocked_lookahead(1.0)
    check("lookahead 吃到立体盒", d is not None and d <= 0.55)


def test_column_inside_body_box_dead_zone_stops() -> None:
    """椅座中柱在立体盒/近场门槛上（y=0.20），必须急停。"""
    lidar = _FakeLidar(5.0, body_points=[_P(0.02, 0.20, 0.28)])
    lidar._acc = AccumulatingSectors()
    lidar._acc.add(0.0, 5.0)
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=3))
    v, w, blocked = guard.guard_velocity(0.10, 0.0)
    check("0.20 m 中柱立即急停", blocked and v == 0.0)


def test_overhead_canopy_stops_when_under_seat() -> None:
    """钻进椅面下方：水平扇区和立体盒都空，头顶点必须急停。"""
    pts = [_P(0.04 * ((i % 7) - 3), 0.16, 0.38) for i in range(14)]
    lidar = _FakeLidar(5.0, body_points=pts)
    lidar._acc = AccumulatingSectors()
    lidar._acc.add(0.0, 5.0)
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=3))
    v, w, blocked = guard.guard_velocity(0.10, 0.0)
    check("头顶椅面急停", blocked and v == 0.0)


def test_track_side_chair_leg_stops() -> None:
    """履带顶上五星椅腿：不在 ±30° 锥里。"""
    lidar = _FakeLidar(5.0, body_points=[_P(0.28, 0.07, 0.12)])
    lidar._acc = AccumulatingSectors()
    lidar._acc.add(0.0, 5.0)
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=3))
    v, w, blocked = guard.guard_velocity(0.10, 0.0)
    check("履带旁椅腿急停", blocked and v == 0.0)


def test_center_cable_clutter_does_not_stop() -> None:
    """雷达正前方低矮线缆不得当成人/椅急停。"""
    lidar = _FakeLidar(5.0, body_points=[_P(0.04, 0.08, 0.05)])
    lidar._acc = AccumulatingSectors()
    lidar._acc.add(0.0, 5.0)
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
    v, w, blocked = guard.guard_velocity(0.10, 0.0)
    check("中轴线缆不急停", not blocked)
    d, kind, n = near_collision_hits([_P(0.04, 0.08, 0.05)])
    check("中轴线缆不进近场", d is None and kind == "" and n == 0)


def test_stray_far_column_point_does_not_hard_stop() -> None:
    """空地单点自扫（0.4 m 一粒）不得按立柱急停，否则 m 几乎走不动。"""
    lidar = _FakeLidar(5.0, body_points=[_P(0.05, 0.40, 0.20)])
    lidar._acc = AccumulatingSectors()
    lidar._acc.add(0.0, 5.0)
    guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
    v, w, blocked = guard.guard_velocity(0.15, 0.0)
    check("空地单点不急停", not blocked)
    check("空地单点接近原速", v >= 0.10)


def test_min_range_vanish_keeps_stop() -> None:
    """中柱进入 Airy 0.1 m 盲区、回波变空：仍保持前进急停，倒车可走。"""
    lidar = _FakeLidar(5.0, body_points=[_P(0.02, 0.20, 0.30)])
    lidar._acc = AccumulatingSectors()
    lidar._acc.add(0.0, 5.0)
    guard = ObstacleGuard(
        lidar, ObstaclePolicy(stop_confirm_frames=1, furniture_latch_s=2.0))
    guard.guard_velocity(0.10, 0.0)
    lidar.latest_frame.points = []
    v, w, blocked = guard.guard_velocity(0.10, 0.0)
    check("盲区消失后前进仍急停", blocked and v == 0.0)
    v2, w2, blocked2 = guard.guard_velocity(-0.10, 0.0)
    check("倒车离开不锁死", not blocked2 and v2 < 0.0)


def test_accum_sector_index_does_not_alias_120_deg() -> None:
    """120 格扇区不得把 120°/240° 叠进 0°。"""
    s = AccumulatingSectors(120)
    s.add(120.0, 0.3)
    check("120° 不出现在正前方", s.nearest_in_range(0.0, 40.0) is None)
    check("120° 仍在左侧", s.nearest_in_range(120.0, 20.0) == 0.3)


def main() -> None:
    print("obstacle safety:")
    test_unmeasured_wall_stops()
    test_gravel_still_slows()
    test_stale_timeout_is_tight()
    test_zero_move_does_not_cry_lidar_lost()
    test_navigator_backup_is_guarded()
    test_body_box_stops_chair_when_sector_is_empty()
    test_chair_at_half_meter_hard_stops()
    test_body_box_hold_survives_one_empty_frame()
    test_lookahead_uses_body_box()
    test_column_inside_body_box_dead_zone_stops()
    test_overhead_canopy_stops_when_under_seat()
    test_track_side_chair_leg_stops()
    test_center_cable_clutter_does_not_stop()
    test_stray_far_column_point_does_not_hard_stop()
    test_min_range_vanish_keeps_stop()
    test_accum_sector_index_does_not_alias_120_deg()
    print("ALL PASS")


if __name__ == "__main__":
    main()
