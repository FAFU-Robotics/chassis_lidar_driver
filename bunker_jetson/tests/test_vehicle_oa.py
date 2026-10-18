"""实车避障包络：570×80 mm、台阶 4 cm、溶洞关家具锁、坑比立障更早停。"""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
_JETSON = _ROOT / "bunker_jetson"
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
if str(_JETSON) not in sys.path:
    sys.path.insert(0, str(_JETSON))

from bunker_mini.lidar import AccumulatingSectors, SelfMaskConfig, filter_self_hardware
from bunker_mini.obstacle import (
    VEHICLE_WIDTH_M,
    VFH_SAFETY_MARGIN_M,
    ObstacleGuard,
    ObstaclePolicy,
    make_obstacle_policy,
    office_obstacle_policy,
)
from bunker_mini.terrain import DEFAULT_STEP_LIMIT_M, TerrainSectorResult
from bunker_mini.vfh import VFHConfig


class _P:
    def __init__(self, x, y, z) -> None:
        self.x, self.y, self.z = x, y, z
        self.distance_m = (x * x + y * y + z * z) ** 0.5


class _FakeLidar:
    def __init__(self, front: float, terrain: TerrainSectorResult | None = None,
                 body_points=None) -> None:
        self._acc = AccumulatingSectors()
        self._acc.add(0.0, front)
        self._acc.add(180.0, 5.0)
        self._terrain = terrain or TerrainSectorResult(angle_deg=0.0)
        self.is_receiving = True
        self.latest_frame = type("F", (), {"points": list(body_points or [])})()

    def nearest_in_range(self, center_deg, width_deg, quantile=0.25):
        return self._acc.nearest_in_range(center_deg, width_deg, quantile)

    def terrain_sector(self, angle_deg, step_limit_m=0.04):
        return self._terrain


def _terrain(**kw) -> TerrainSectorResult:
    return TerrainSectorResult(angle_deg=0.0, **kw)


class VehicleEnvelopeTest(unittest.TestCase):
    def test_width_and_step_match_measured_chassis(self) -> None:
        self.assertAlmostEqual(VEHICLE_WIDTH_M, 0.57)
        self.assertAlmostEqual(VFH_SAFETY_MARGIN_M, 0.08)
        self.assertAlmostEqual(DEFAULT_STEP_LIMIT_M, 0.04)
        cfg = VFHConfig()
        self.assertAlmostEqual(cfg.vehicle_width_m, 0.57)
        self.assertAlmostEqual(cfg.safety_margin_m, 0.08)

    def test_cave_policy_disables_furniture_latch(self) -> None:
        p = ObstaclePolicy()
        self.assertEqual(p.furniture_latch_s, 0.0)
        self.assertLess(p.body_stop_distance_m, 0.50)
        self.assertAlmostEqual(p.pit_stop_m, 0.60)
        self.assertAlmostEqual(p.stop_distance_m, 0.35)

    def test_office_policy_restores_chair_lock(self) -> None:
        p = office_obstacle_policy()
        self.assertGreaterEqual(p.furniture_latch_s, 0.8)
        self.assertAlmostEqual(p.body_stop_distance_m, 0.55)


class SceneEnvTest(unittest.TestCase):
    def setUp(self) -> None:
        self._old = os.environ.get("BUNKER_OA_SCENE")

    def tearDown(self) -> None:
        if self._old is None:
            os.environ.pop("BUNKER_OA_SCENE", None)
        else:
            os.environ["BUNKER_OA_SCENE"] = self._old

    def test_office_env_enables_latch(self) -> None:
        os.environ["BUNKER_OA_SCENE"] = "office"
        p = make_obstacle_policy()
        self.assertGreaterEqual(p.furniture_latch_s, 0.8)
        self.assertAlmostEqual(p.body_stop_distance_m, 0.55)

    def test_cave_env_keeps_latch_off(self) -> None:
        os.environ["BUNKER_OA_SCENE"] = "cave"
        p = make_obstacle_policy()
        self.assertEqual(p.furniture_latch_s, 0.0)


class PitVsStandingTest(unittest.TestCase):
    def test_measured_pit_at_half_meter_stops(self) -> None:
        """立障 stop 在 ~0.41 m（0.10 m/s），坑必须在 0.50 m 就停。"""
        lidar = _FakeLidar(
            1.5,
            TerrainSectorResult(
                angle_deg=0.0, obstacle_distance_m=0.50, max_height_m=0.20),
        )
        guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
        v, _w, blocked = guard.guard_velocity(0.10, 0.0)
        self.assertTrue(blocked)
        self.assertEqual(v, 0.0)

    def test_standing_object_at_half_meter_does_not_hard_stop_in_cave(self) -> None:
        """溶洞：0.50 m 立障只限速，不按办公椅 0.55 m 闸。"""
        lidar = _FakeLidar(0.50, _terrain(max_height_m=0.40))
        guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
        v, _w, blocked = guard.guard_velocity(0.10, 0.0)
        self.assertFalse(blocked)
        self.assertGreater(v, 0.0)
        self.assertLess(v, 0.10)

    def test_office_chair_at_half_meter_stops(self) -> None:
        pts = [_P(-0.08 + 0.04 * i, 0.46, 0.48) for i in range(5)]
        lidar = _FakeLidar(5.0, body_points=pts)
        lidar._acc = AccumulatingSectors()
        lidar._acc.add(0.0, 5.0)
        guard = ObstacleGuard(
            lidar, office_obstacle_policy(stop_confirm_frames=1))
        v, _w, blocked = guard.guard_velocity(0.10, 0.0)
        self.assertTrue(blocked)
        self.assertEqual(v, 0.0)

    def test_gravel_3cm_still_slows(self) -> None:
        lidar = _FakeLidar(0.25, _terrain(max_height_m=0.03))
        guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
        v, _w, blocked = guard.guard_velocity(0.3, 0.0)
        self.assertFalse(blocked)
        self.assertLess(v, 0.3)

    def test_unmeasured_near_return_is_not_gravel(self) -> None:
        lidar = _FakeLidar(0.20, _terrain(max_height_m=0.0))
        guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
        v, _w, blocked = guard.guard_velocity(0.3, 0.0)
        self.assertTrue(blocked)
        self.assertEqual(v, 0.0)

    def test_pit_lookahead_at_80cm_slows_not_stop(self) -> None:
        lidar = _FakeLidar(
            1.5,
            TerrainSectorResult(
                angle_deg=0.0, obstacle_distance_m=0.80, max_height_m=0.20),
        )
        guard = ObstacleGuard(lidar, ObstaclePolicy(stop_confirm_frames=1))
        v, _w, blocked = guard.guard_velocity(0.3, 0.0)
        self.assertFalse(blocked)
        self.assertLess(v, 0.3)


class SelfMaskFrontLipTest(unittest.TestCase):
    def test_front_lip_box_filters_mount(self) -> None:
        cfg = SelfMaskConfig()
        self.assertLessEqual(cfg.y_max_m, 0.18)
        inside = _P(0.10, 0.10, 0.20)
        outside = _P(0.10, 0.50, 0.20)
        out = filter_self_hardware([inside, outside], cfg)
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0].y, 0.50)


if __name__ == "__main__":
    unittest.main()
