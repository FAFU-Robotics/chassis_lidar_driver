#!/usr/bin/env python3
"""Vision target multi-frame confirmation tests.

验证 agent 停车多帧确认：
  * 连续 N 帧同一位置命中 → 确认返回
  * 中间缺帧 / 位置跳变 → 判定误检返回 None

用法：
    python3 -u _vision_confirm_test.py
"""

import sys
import threading
import unittest

from bunker_mini.agent import BunkerMiniAgent
from bunker_mini.vision import TargetEstimate


class SeqDetector:
    """按帧返回预设结果的假检测器。"""

    def __init__(self, per_frame: list) -> None:
        self._seq = list(per_frame)
        self._i = 0

    def detect(self, points):
        if self._i < len(self._seq):
            r = self._seq[self._i]
            self._i += 1
            return r
        return []


def _est(distance: float, bearing: float) -> TargetEstimate:
    return TargetEstimate(distance_m=distance, bearing_deg=bearing,
                          point_count=5, reflectivity=220)


class _BareAgent(BunkerMiniAgent):
    """只有 _confirm_target 所需接口的轻量宿主（复用 Agent 的实现）。"""

    def __init__(self, seq: list) -> None:
        self._det = SeqDetector(seq)
        self._drive_calls: list[tuple[float, float]] = []
        self._stop = threading.Event()

    def _drive(self, v: float, w: float) -> None:
        self._drive_calls.append((v, w))

    def _detect_target(self, detector, lidar, name="target"):
        ests = detector.detect(None)
        return ests[0] if ests else None


class VisionConfirmTest(unittest.TestCase):
    def test_confirmed_after_three_consistent_frames(self) -> None:
        agent = _BareAgent([
            [_est(1.0, 5.0)],
            [_est(1.05, 6.0)],
            [_est(1.02, 5.5)],
        ])
        est = agent._confirm_target(agent._det, None, agent._stop, frames=3)
        self.assertIsNotNone(est)
        self.assertAlmostEqual(est.distance_m, 1.02)
        # 停车指令已下发
        self.assertTrue(any(v == 0.0 for v, _ in agent._drive_calls))

    def test_rejected_when_frame_missing(self) -> None:
        agent = _BareAgent([
            [_est(1.0, 5.0)],
            [],
            [],
            [],
        ])
        est = agent._confirm_target(agent._det, None, agent._stop, frames=3)
        self.assertIsNone(est)

    def test_rejected_on_distance_jump(self) -> None:
        agent = _BareAgent([
            [_est(1.0, 5.0)],
            [_est(1.8, 5.0)],   # 距离跳变 > 0.25m
            [_est(1.0, 5.0)],
        ])
        est = agent._confirm_target(agent._det, None, agent._stop, frames=3)
        self.assertIsNone(est)

    def test_rejected_on_bearing_jump(self) -> None:
        agent = _BareAgent([
            [_est(1.0, 5.0)],
            [_est(1.0, 40.0)],  # 方位跳变 > 10°
            [_est(1.0, 5.0)],
        ])
        est = agent._confirm_target(agent._det, None, agent._stop, frames=3)
        self.assertIsNone(est)

    def test_stop_event_aborts(self) -> None:
        agent = _BareAgent([[_est(1.0, 5.0)]] * 10)
        agent._stop.set()
        est = agent._confirm_target(agent._det, None, agent._stop, frames=3)
        self.assertIsNone(est)


if __name__ == "__main__":
    suite = unittest.TestLoader().loadTestsFromTestCase(VisionConfirmTest)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
