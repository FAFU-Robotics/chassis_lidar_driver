"""goto 优化：A* 膨胀刷新、改目标、轮距标定、到达残差。"""

from __future__ import annotations

import math
import tempfile
import time
import unittest
from pathlib import Path

from bunker_mini.global_planner import GlobalPlanner
from bunker_mini.navigator import (
    NavigateConfig,
    Navigator,
    OdometryPose,
    load_wheelbase_m,
    save_wheelbase_m,
    scale_wheelbase,
)
from bunker_mini.occupancy import OCCUPIED, OccupancyGrid, PSEUDO_MAP_TTL_S


class _Ctrl:
    def __init__(self) -> None:
        self.calls: list = []

    def set_velocity(self, v, w):
        self.calls.append(("ctrl", v, w))

    def stop_motion(self):
        self.calls.append(("stop",))


class WheelbaseMathTest(unittest.TestCase):
    def test_scale_overshoot_means_wheelbase_too_small(self) -> None:
        # 积分 90°、地面 80° → 轮距应放大
        wb = scale_wheelbase(0.50, 90.0, 80.0)
        self.assertAlmostEqual(wb, 0.50 * 90.0 / 80.0, places=4)

    def test_scale_rejects_tiny_actual_heading(self) -> None:
        with self.assertRaises(ValueError):
            scale_wheelbase(0.50, 5.0, 4.0)

    def test_roundtrip_file(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / "wheelbase.local"
            save_wheelbase_m(0.482, path)
            self.assertAlmostEqual(load_wheelbase_m(path=path), 0.482, places=3)


class OccupancyTtlTest(unittest.TestCase):
    def test_default_ttl_is_decay_window(self) -> None:
        g = OccupancyGrid()
        self.assertGreater(g.ttl_s, 0.0)
        self.assertAlmostEqual(g.ttl_s, PSEUDO_MAP_TTL_S)


class PlannerCacheTest(unittest.TestCase):
    def test_plan_drops_stale_inflation(self) -> None:
        grid = OccupancyGrid(resolution_m=0.1)
        with grid._lock:
            grid._cells[(10, 0)] = OCCUPIED
        planner = GlobalPlanner(grid, inflation_m=0.3)
        planner.plan(0.0, 0.0, 3.0, 0.0)
        with grid._lock:
            grid._cells.clear()
            grid._cells[(10, 10)] = OCCUPIED
        path = planner.plan(0.0, 0.0, 3.0, 0.0)
        self.assertIsNotNone(path)
        self.assertEqual(len(path), 2)


class NavigatorReplaceTest(unittest.TestCase):
    def test_second_goto_replaces_goal(self) -> None:
        nav = Navigator(
            _Ctrl(), guard=None, wheelbase_m=0.5,
            config=NavigateConfig(update_interval_s=0.02),
        )
        self.assertTrue(nav.goto(5.0, 0.0))
        self.assertTrue(nav.is_navigating)
        self.assertTrue(nav.goto(0.40, 0.10))
        self.assertEqual(nav.goal, (0.40, 0.10))
        nav.stop()
        self.assertFalse(nav.is_navigating)

    def test_arrived_records_residual(self) -> None:
        nav = Navigator(
            _Ctrl(), guard=None, wheelbase_m=0.5,
            config=NavigateConfig(
                max_linear_m_s=0.4,
                goal_tolerance_m=0.08,
                update_interval_s=0.01,
                final_yaw_tolerance_rad=0.2,
            ),
        )
        arrived: list[bool] = []
        nav.goto(0.30, 0.0, on_arrived=lambda: arrived.append(True))
        mm = 0
        deadline = time.monotonic() + 4.0
        while nav.is_navigating and time.monotonic() < deadline:
            mm += 12
            nav.feed_odometry(mm, mm)
            time.sleep(0.008)
        nav.stop()
        self.assertTrue(arrived, "should reach a 0.30 m goal")
        res = nav.last_result
        self.assertIsNotNone(res)
        self.assertTrue(res["arrived"])
        self.assertLess(res["errM"], 0.12)

    def test_set_wheelbase_changes_yaw_gain(self) -> None:
        pose = OdometryPose(0.50)
        pose.update(0, 0)
        pose.update(0, 157)  # dr-dl = 0.157 m → yaw = 0.157/0.5 ≈ 0.314 rad
        yaw_wide = pose.pose.yaw
        pose2 = OdometryPose(0.40)
        pose2.update(0, 0)
        pose2.update(0, 157)
        self.assertGreater(abs(pose2.pose.yaw), abs(yaw_wide))

    def test_set_yaw_keeps_wheel_baseline(self) -> None:
        pose = OdometryPose(0.50)
        pose.update(0, 0)
        pose.update(100, 100)
        x0 = pose.pose.x
        pose.set_yaw_deg(10.0)
        self.assertAlmostEqual(pose.pose.x, x0, places=5)
        self.assertAlmostEqual(pose.pose.yaw_deg, 10.0, places=3)
        pose.update(120, 120)
        # 若重置了轮跳基准，这一拍只会记下 120 而不积分。
        moved = math.hypot(pose.pose.x - x0, pose.pose.y)
        self.assertAlmostEqual(moved, 0.02, places=4)


class NearGoalLookaheadTest(unittest.TestCase):
    def test_near_goal_does_not_slam_steer_from_lookahead(self) -> None:
        class _Guard:
            def front_blocked_lookahead(self, lookahead_m):
                return 0.50

            def steer_away_deg(self):
                return 90.0

            def guard_velocity(self, v, w):
                return v, w, False

        ctrl = _Ctrl()
        nav = Navigator(
            ctrl, guard=_Guard(), wheelbase_m=0.5,
            config=NavigateConfig(
                update_interval_s=0.02,
                max_angular_rad_s=0.60,
                approach_lock_m=0.25,
                goal_tolerance_m=0.08,
                max_linear_m_s=0.30,
            ),
        )
        nav.goto(0.12, 0.0)
        deadline = time.monotonic() + 1.2
        mm = 0
        while nav.is_navigating and time.monotonic() < deadline:
            mm += 4
            nav.feed_odometry(mm, mm)
            time.sleep(0.01)
        nav.stop()
        w_vals = [abs(c[2]) for c in ctrl.calls if c[0] == "ctrl"]
        self.assertTrue(w_vals)
        self.assertLess(max(w_vals), 0.50)


class OdomSpikeAndSlewTest(unittest.TestCase):
    def test_odometry_skips_meter_scale_spike(self) -> None:
        pose = OdometryPose(0.50)
        pose.update(0, 0)
        pose.update(80, 80)
        x0 = pose.pose.x
        pose.update(80 + 1200, 80 + 1200)  # 1.2 m 一拍，应丢弃
        self.assertAlmostEqual(pose.pose.x, x0, places=5)
        pose.update(80 + 1220, 80 + 1220)  # 之后 20 mm 正常累加
        self.assertAlmostEqual(pose.pose.x, x0 + 0.02, places=5)

    def test_nav_slew_does_not_jump_to_cap(self) -> None:
        ctrl = _Ctrl()
        nav = Navigator(
            ctrl, guard=None, wheelbase_m=0.5,
            config=NavigateConfig(
                update_interval_s=0.02,
                v_slew_m_s2=0.45,
                max_linear_m_s=0.30,
            ),
        )
        nav._goal_speed = 0.15
        nav._set_vel(0.15, 0.0)
        self.assertTrue(ctrl.calls)
        v0 = ctrl.calls[0][1]
        self.assertLess(v0, 0.04)
        self.assertGreater(v0, 0.0)
        for _ in range(40):
            nav._set_vel(0.15, 0.0)
        self.assertAlmostEqual(ctrl.calls[-1][1], 0.15, places=3)
        nav._set_vel(0.0, 0.0)
        self.assertEqual(ctrl.calls[-1][1], 0.0)


class CommandParseTest(unittest.TestCase):
    def test_goto_payload_yaw_and_wheelbase(self) -> None:
        from bunker_mini.agent import Command
        c = Command.from_payload({
            "action": "goto", "x": 1.2, "y": -0.4, "speed": 0.15, "yawDeg": 90,
        })
        self.assertTrue(c.goal_yaw_set)
        self.assertAlmostEqual(c.yaw_deg, 90.0)
        self.assertAlmostEqual(c.speed, 0.15)
        w = Command.from_payload({"action": "set_wheelbase", "wheelbaseM": 0.46})
        self.assertAlmostEqual(w.wheelbase_m, 0.46)
        cal = Command.from_payload({"action": "calibrate_wheelbase", "yawDeg": 80})
        self.assertTrue(cal.goal_yaw_set)
        self.assertAlmostEqual(cal.yaw_deg, 80.0)


if __name__ == "__main__":
    unittest.main()
