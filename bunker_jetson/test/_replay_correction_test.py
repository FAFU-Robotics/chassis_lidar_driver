#!/usr/bin/env python3
"""Closed-loop playback correction tests.

强化回放纠偏：
  * 横向偏差 → 反向角速度修正（车偏左 → 右转回线）
  * 航向偏差大时横偏项衰减（先转头再纠线，防 S 形振荡）
  * 横向偏差时间窗积分补偿恒定漂移
  * 双重限幅防 windup

用法：
    python3 -u _replay_correction_test.py
"""

import sys
import time
import unittest

from bunker_mini.navigator import Pose2D
from bunker_mini.tracker import PlaybackCorrectionConfig, TrackPlayer


def _bare_player(cfg: PlaybackCorrectionConfig) -> TrackPlayer:
    player = object.__new__(TrackPlayer)
    player._correction = cfg
    player._cross_int = 0.0
    player._cross_int_t = None
    player._last_correction = 0.0
    player._lock = None
    return player


class ReplayCorrectionTest(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = PlaybackCorrectionConfig()
        self.player = _bare_player(self.cfg)

    def _apply(self, expected: Pose2D, actual_pose: Pose2D,
               base_w: float = 0.0) -> float:
        # 绕过 self._lock（bare 对象无锁）
        return self.player._apply_correction(expected, actual_pose, base_w)

    def test_no_correction_when_disabled(self) -> None:
        cfg = PlaybackCorrectionConfig(enabled=False)
        player = _bare_player(cfg)
        w = player._apply_correction(
            Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.2, 0.0), base_w=0.15)
        self.assertEqual(w, 0.15)

    def test_cross_track_error_steers_back(self) -> None:
        # 期望沿 +x，车偏左（y 更大）→ 应右转（w < base）
        w = self._apply(Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.05, 0.0), base_w=0.0)
        self.assertLess(w, 0.0)
        self.assertGreaterEqual(w, -self.cfg.max_correction_rad_s)

    def test_heading_error_steers_correct_direction(self) -> None:
        # 车头偏左 0.3 rad → 应右转（w < base）
        w = self._apply(Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.0, 0.3), base_w=0.0)
        self.assertLess(w, 0.0)

    def test_heading_priority_decays_cross_term(self) -> None:
        # 大横偏 + 大航向偏：横偏项被衰减，修正主要由航向项主导（同方向）。
        # 车头偏右 0.8 rad、车偏右 0.1 m：都应左转（w > base）
        w = self._apply(Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, -0.1, -0.8), base_w=0.0)
        self.assertGreater(w, 0.0)

    def test_integral_accumulates_over_time(self) -> None:
        # 恒定横偏 0.03 m，持续 2 秒：积分项逐渐贡献
        for i in range(100):
            now = time.monotonic()
            if self.player._cross_int_t is None:
                self.player._cross_int_t = now - 0.02  # 首帧 dt=0
            else:
                # 模拟 20ms 调用间隔
                self.player._cross_int_t = self.player._cross_int_t + 0.02
            self._apply(Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.03, 0.0))
        self.assertNotAlmostEqual(self.player._cross_int, 0.0)
        # 积分方向：车偏左 → 积分项为负 → 修正右转
        self.assertLess(self.player._cross_int, 0.0)

    def test_integral_clamped(self) -> None:
        # 极限横偏长时间累积 → 积分项不超 clamp 上限
        for i in range(500):
            if self.player._cross_int_t is None:
                self.player._cross_int_t = time.monotonic() - 0.02
            else:
                self.player._cross_int_t = self.player._cross_int_t + 0.02
            self._apply(Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.5, 0.0))
        limit = self.cfg.integral_clamp / max(self.cfg.cross_track_integral_gain, 1e-3)
        self.assertLessEqual(abs(self.player._cross_int), limit + 1e-6)

    def test_correction_capped(self) -> None:
        w = self._apply(Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 5.0, 3.0), base_w=0.0)
        self.assertLessEqual(w, self.cfg.max_correction_rad_s + 1e-6)
        self.assertGreaterEqual(w, -self.cfg.max_correction_rad_s - 1e-6)

    def test_large_gap_resets_integral(self) -> None:
        self.player._cross_int = 0.5
        # 超过积分窗的空隙 → 积分清零
        self.player._cross_int_t = time.monotonic() - 10.0
        self._apply(Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.03, 0.0))
        self.assertAlmostEqual(self.player._cross_int, 0.0, delta=1e-9)


if __name__ == "__main__":
    suite = unittest.TestLoader().loadTestsFromTestCase(ReplayCorrectionTest)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
