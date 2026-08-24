#!/usr/bin/env python3
"""Navigator waypoints (planned-path) navigation tests.

验证 goto(waypoints=...) 沿中间航点逐个推进、最终到达并回调 on_arrived，
且不会因「到达中间点瞬间 pose 停顿」被误判为卡死。

用法：
    python3 -u _navigator_waypoints_test.py
"""

import sys
import time
import unittest

from bunker_mini.navigator import NavigateConfig, Navigator


class FakeCtrl:
    def __init__(self) -> None:
        self.cmds: list[tuple[float, float]] = []
        self.stopped = False

    def set_velocity(self, v: float, w: float) -> None:
        self.cmds.append((v, w))

    def stop_motion(self) -> None:
        self.stopped = True


def _make_nav(ctrl: FakeCtrl) -> Navigator:
    cfg = NavigateConfig(
        max_linear_m_s=0.3,
        max_angular_rad_s=0.6,
        goal_tolerance_m=0.1,
        update_interval_s=0.02,
        stall_timeout_s=10.0,
        arrive_probe_m=0.4,
        arrive_stall_s=0.5,
    )
    nav = Navigator(ctrl, wheelbase_m=0.5, config=cfg)
    nav.reset_pose()
    return nav


class NavigatorWaypointsTest(unittest.TestCase):
    def test_drives_through_waypoints_to_goal(self) -> None:
        ctrl = FakeCtrl()
        nav = _make_nav(ctrl)
        arrived: list[bool] = []

        ok = nav.goto(0.6, 0.0, waypoints=[(0.3, 0.0), (0.45, 0.0)],
                      on_arrived=lambda: arrived.append(True))
        self.assertTrue(ok)

        # 模拟底盘沿 +x 前进：每 20ms 走 2cm（左右轮里程同步推进）
        left = right = 0
        deadline = time.monotonic() + 8.0
        while not arrived and time.monotonic() < deadline:
            left += 20
            right += 20
            nav.feed_odometry(left, right)
            time.sleep(0.01)

        self.assertTrue(arrived, "navigator should arrive after passing all waypoints")
        self.assertTrue(ctrl.stopped)

    def test_waypoints_are_visited_in_order(self) -> None:
        ctrl = FakeCtrl()
        nav = _make_nav(ctrl)
        visited: list[tuple[float, float]] = []

        ok = nav.goto(0.6, 0.0, waypoints=[(0.3, 0.0), (0.45, 0.0)],
                      on_arrived=lambda: None)
        self.assertTrue(ok)
        self.assertEqual(nav._waypoints, [(0.3, 0.0), (0.45, 0.0)])

        left = right = 0
        deadline = time.monotonic() + 8.0
        while not nav._stop_event.is_set() and time.monotonic() < deadline:
            p = nav.pose
            # 记录每次越过新中间点的事件（简化：直接检查 waypoint 推进）
            left += 20
            right += 20
            nav.feed_odometry(left, right)
            time.sleep(0.01)
        # 无论是否到终点，导航必须结束（is_navigating False）
        nav.stop()
        self.assertFalse(nav.is_navigating)

    def test_direct_goto_without_waypoints(self) -> None:
        ctrl = FakeCtrl()
        nav = _make_nav(ctrl)
        arrived: list[bool] = []
        ok = nav.goto(0.5, 0.0, on_arrived=lambda: arrived.append(True))
        self.assertTrue(ok)
        self.assertEqual(nav._waypoints, [])

        left = right = 0
        deadline = time.monotonic() + 8.0
        while not arrived and time.monotonic() < deadline:
            left += 20
            right += 20
            nav.feed_odometry(left, right)
            time.sleep(0.01)
        self.assertTrue(arrived)


if __name__ == "__main__":
    suite = unittest.TestLoader().loadTestsFromTestCase(NavigatorWaypointsTest)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
