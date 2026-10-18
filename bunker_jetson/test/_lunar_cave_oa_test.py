#!/usr/bin/env python3
"""月球溶洞避障覆盖：地形剖面 + 守卫，不依赖真雷达。

覆盖碎石/大石/台阶/坑/坡/钟乳石/高顶/悬崖/黑洞，以及「室内椅子补丁
不得在溶洞高顶上误刹」。用 python3 直接跑，不依赖 pytest。
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from bunker_mini.lidar import AccumulatingSectors, LidarPoint
from bunker_mini.obstacle import ObstacleGuard, ObstaclePolicy, hull_clearance
from bunker_mini.terrain import TerrainProfile, TerrainSectorResult


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


def check(name: str, cond: bool, detail: str = "") -> None:
    if cond:
        print(f"  PASS  {name}")
        return
    print(f"  FAIL  {name}  {detail}")
    raise SystemExit(1)


def _pt(az_deg: float, dist_m: float, z: float) -> LidarPoint:
    az_r = math.radians(az_deg)
    return LidarPoint(
        azimuth_deg=az_deg, vertical_deg=0.0, distance_m=dist_m,
        reflectivity=80, channel=1,
        x=dist_m * math.sin(az_r), y=dist_m * math.cos(az_r), z=z,
    )


def _ground(dists: list[float], z: float = -0.35) -> list[LidarPoint]:
    return [_pt(0.0, d, z) for d in dists]


def test_gravel_slows() -> None:
    lidar = _FakeLidar(
        0.25, TerrainSectorResult(angle_deg=0.0, max_height_m=0.03))
    v, w, blocked = ObstacleGuard(
        lidar, ObstaclePolicy(stop_confirm_frames=1)).guard_velocity(0.3, 0.0)
    check("碎石不急停", not blocked)
    check("碎石限速", 0.0 < v < 0.3)


def test_rock_wall_stops() -> None:
    lidar = _FakeLidar(
        0.22,
        TerrainSectorResult(
            angle_deg=0.0, max_height_m=0.40, obstacle_distance_m=0.22),
    )
    v, w, blocked = ObstacleGuard(
        lidar, ObstaclePolicy(stop_confirm_frames=1)).guard_velocity(0.3, 0.0)
    check("近处岩壁急停", blocked and v == 0.0)


def test_step_profile_blocked() -> None:
    prof = TerrainProfile()
    prof.add_frame(_ground([0.2, 0.3, 0.4, 0.5, 0.6]) + _ground(
        [1.0, 1.1, 1.2], z=-0.18))
    r = prof.sector(0.0, 0.07)
    check("台阶地形不可通行", r.blocked and r.obstacle_distance_m is not None)


def test_pit_negative() -> None:
    prof = TerrainProfile()
    prof.add_frame(_ground([0.2, 0.3, 0.4, 0.5]) + _ground(
        [1.0, 1.1, 1.2], z=-0.55))
    r = prof.sector(0.0, 0.07)
    check("坑沿判下坠", r.negative_obstacle_distance_m is not None)


def test_slope_not_blocked() -> None:
    prof = TerrainProfile()
    pts = [_pt(0.0, d, -0.35 + 0.25 * d)
           for d in (0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2)]
    prof.add_frame(pts)
    r = prof.sector(0.0, 0.07)
    check("连续缓坡不当墙", r.is_slope and not r.blocked)


def test_hanging_stalactite_profile() -> None:
    """地板还在，0.7 m 处钟乳石伸进车体高度。"""
    pts = _ground([0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9])
    pts += [_pt(0.0, 0.70, 0.22), _pt(0.0, 0.70, -0.35)]
    prof = TerrainProfile()
    prof.add_frame(pts)
    r = prof.sector(0.0, 0.07)
    check("钟乳石地形不可通行", r.blocked and r.obstacle_distance_m is not None)


def test_cliff_void_is_drop() -> None:
    """近处平坦地面，0.8 m 后直到 2.5 m 都空 = 溶洞口。"""
    prof = TerrainProfile()
    prof.add_frame(_ground([0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75]))
    r = prof.sector(0.0, 0.07)
    check("地面突然消失判悬崖", r.negative_obstacle_distance_m is not None)


def test_hull_stalactite_stops() -> None:
    lidar = _FakeLidar(5.0, body_points=[_P(0.02, 0.40, 0.22)])
    lidar._acc = AccumulatingSectors()
    lidar._acc.add(0.0, 5.0)
    v, w, blocked = ObstacleGuard(
        lidar, ObstaclePolicy(stop_confirm_frames=3)).guard_velocity(0.10, 0.0)
    check("车体高度钟乳石急停", blocked and v == 0.0)
    check("hull_clearance 看见 0.40 m", hull_clearance(
        lidar.latest_frame.points) == 0.40)


def test_high_cave_ceiling_does_not_hard_stop() -> None:
    """溶洞顶板在雷达上方 0.9 m：车钻得过，不得当椅面急停。"""
    pts = [_P(0.08 * ((i % 5) - 2), 0.20, 0.90) for i in range(10)]
    lidar = _FakeLidar(5.0, body_points=pts)
    lidar._acc = AccumulatingSectors()
    lidar._acc.add(0.0, 5.0)
    lidar._terrain = TerrainSectorResult(angle_deg=0.0)
    v, w, blocked = ObstacleGuard(
        lidar, ObstaclePolicy(stop_confirm_frames=1)).guard_velocity(0.10, 0.0)
    check("高顶不急停", not blocked)
    check("高顶可放行接近原速", v >= 0.08)


def test_void_unclear_slows() -> None:
    lidar = _FakeLidar(5.0)
    lidar._acc = AccumulatingSectors()
    lidar._acc.add(180.0, 5.0)
    lidar._terrain = TerrainSectorResult(angle_deg=0.0, unclear=True)
    v, w, blocked = ObstacleGuard(
        lidar, ObstaclePolicy(stop_confirm_frames=1)).guard_velocity(0.20, 0.0)
    check("黑洞不急停", not blocked)
    check("黑洞不得全速", 0.0 < v < 0.20)


def test_zero_tilt_cannot_see_floor() -> None:
    """Airy 垂直 0~90°、俯仰 0：朝下没有激光，地面/坑根本不进地形格。"""
    prof = TerrainProfile()
    r = prof.sector(0.0, 0.07)
    check("无地面回波时地形 unclear", r.unclear and not r.blocked)


def test_chair_canopy_still_stops() -> None:
    pts = [_P(0.04 * ((i % 7) - 3), 0.16, 0.38) for i in range(14)]
    lidar = _FakeLidar(5.0, body_points=pts)
    lidar._acc = AccumulatingSectors()
    lidar._acc.add(0.0, 5.0)
    v, w, blocked = ObstacleGuard(
        lidar, ObstaclePolicy(stop_confirm_frames=3)).guard_velocity(0.10, 0.0)
    check("椅面高度仍急停", blocked and v == 0.0)


def test_empty_mount_clutter_does_not_stop() -> None:
    """空地：雷达附近线缆/电源不得报「钻进桌椅」。"""
    pts = [
        _P(0.04, -0.05, 0.28), _P(-0.06, 0.02, 0.22), _P(0.08, 0.10, 0.20),
        _P(-0.10, 0.12, 0.18), _P(0.02, 0.08, 0.32), _P(0.12, 0.06, 0.21),
        _P(-0.03, 0.00, 0.35), _P(0.15, 0.14, 0.19), _P(0.00, 0.05, 0.24),
        _P(-0.14, 0.09, 0.23),
    ]
    lidar = _FakeLidar(5.0, body_points=pts)
    lidar._acc = AccumulatingSectors()
    lidar._acc.add(0.0, 5.0)
    v, w, blocked = ObstacleGuard(
        lidar, ObstaclePolicy(stop_confirm_frames=1)).guard_velocity(0.10, 0.0)
    check("空地自扫不急停", not blocked)
    check("空地可放行", v >= 0.08)


def main() -> None:
    print("lunar cave OA:")
    test_gravel_slows()
    test_rock_wall_stops()
    test_step_profile_blocked()
    test_pit_negative()
    test_slope_not_blocked()
    test_hanging_stalactite_profile()
    test_cliff_void_is_drop()
    test_hull_stalactite_stops()
    test_high_cave_ceiling_does_not_hard_stop()
    test_void_unclear_slows()
    test_zero_tilt_cannot_see_floor()
    test_chair_canopy_still_stops()
    test_empty_mount_clutter_does_not_stop()
    print("ALL PASS")


if __name__ == "__main__":
    main()
