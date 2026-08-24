#!/usr/bin/env python3
"""Navigator turn-tuning tests (deadband + turn deceleration).

验证：
  * 航向在死区内 → 不转向（w=0）
  * 大转角（90°）→ 先减速转向（v 压低、w 饱和）

用法：
    python3 -u _nav_turn_test.py
"""

import math
import sys
import time
import unittest

from bunker_mini.navigator import NavigateConfig, Navigator


class FakeCtrl:
    def __init__(self) -> None:
        self.cmds: list[tuple[float, float]] = []

    def set_velocity(self, v: float, w: float) -> None:
        self.cmds.append((v, w))

    def stop_motion(self) -> None:
        pass


def _first_cmd(yaw_rad: float, goal=(1.0, 0.0)) -> tuple[float, float]:
    ctrl = FakeCtrl()
    nav = Navigator(ctrl, wheelbase_m=0.5,
                    config=NavigateConfig(update_interval_s=0.02))
    nav._pose.reset(0.0, 0.0, yaw_rad)
    nav.goto(*goal)
    deadline = time.monotonic() + 1.0
    while not ctrl.cmds and time.monotonic() < deadline:
        time.sleep(0.01)
    nav.stop()
    return ctrl.cmds[0] if ctrl.cmds else (0.0, 0.0)


class NavTurnTest(unittest.TestCase):
    def test_deadband_no_turn_when_aligned(self) -> None:
        # 车朝 +x，目标在 +x → 不转向
        v, w = _first_cmd(0.0)
        self.assertAlmostEqual(w, 0.0, delta=0.05)
        self.assertGreater(v, 0.0)

    def test_small_error_inside_deadband(self) -> None:
        # 目标在正前方，仅 3° 航向误差（< 5° 死区）→ 不转向
        v, w = _first_cmd(math.radians(3.0))
        self.assertAlmostEqual(w, 0.0, delta=0.05)

    def test_sharp_turn_is_in_place(self) -> None:
        # 车朝 +y（yaw=90°），目标在 +x → 90° 转角：原地转正，禁止斜着蹭
        v, w = _first_cmd(math.pi / 2.0)
        cfg = NavigateConfig()
        self.assertAlmostEqual(abs(w), cfg.max_angular_rad_s, delta=0.05)
        self.assertAlmostEqual(v, 0.0, delta=0.02)

    def test_moderate_turn_keeps_speed(self) -> None:
        # 20° 转角：低于 align_in_place ~23°，v 不受大影响（仍转，w 非零）
        v, w = _first_cmd(math.radians(20.0))
        cfg = NavigateConfig()
        full = cfg.max_linear_m_s * 1.0
        self.assertGreater(v, full * 0.5)
        self.assertNotAlmostEqual(w, 0.0, delta=0.01)

    def test_final_heading_align_after_xy(self) -> None:
        # 已在目标点但车身偏 90°：不得立刻 arrived，应原地转正
        ctrl = FakeCtrl()
        cfg = NavigateConfig(update_interval_s=0.02, goal_tolerance_m=0.08)
        nav = Navigator(ctrl, wheelbase_m=0.5, config=cfg)
        nav._pose.reset(1.0, 0.0, math.pi / 2.0)
        arrived: list[bool] = []
        nav.goto(1.0, 0.0, on_arrived=lambda: arrived.append(True),
                 goal_yaw=0.0)
        deadline = time.monotonic() + 1.0
        while not ctrl.cmds and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertFalse(arrived)
        self.assertTrue(ctrl.cmds)
        v, w = ctrl.cmds[0]
        self.assertAlmostEqual(v, 0.0, delta=0.02)
        self.assertLess(w, 0.0)  # 从 +90° 转到 0° → 右转
        nav.stop()

    def test_far_stall_does_not_count_as_arrived(self) -> None:
        # 距目标 0.35m 且不动：旧 0.4m 探针会误报到达；现在不得 arrived
        ctrl = FakeCtrl()
        cfg = NavigateConfig(
            update_interval_s=0.02,
            arrive_probe_m=0.16,
            arrive_stall_s=0.3,
            goal_tolerance_m=0.08,
        )
        nav = Navigator(ctrl, wheelbase_m=0.5, config=cfg)
        nav._pose.reset(0.0, 0.0, 0.0)
        arrived: list[bool] = []
        nav.goto(0.35, 0.0, on_arrived=lambda: arrived.append(True))
        time.sleep(0.6)
        self.assertFalse(arrived)
        self.assertTrue(nav.is_navigating)
        nav.stop()


if __name__ == "__main__":
    suite = unittest.TestLoader().loadTestsFromTestCase(NavTurnTest)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
