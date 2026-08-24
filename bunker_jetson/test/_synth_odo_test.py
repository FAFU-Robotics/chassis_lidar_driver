#!/usr/bin/env python3
"""Synthetic odometer tests for the BUNKER MINI remote-control recording fix.

问题：遥控器驾驶时 0x311 里程恒为 0（0x221 速度正常），导致 r/f/fb 录制出
"零里程"轨迹、无法精确回程。修复：控制器用 0x221 速度积分出合成里程，作为
0x311 缺失/卡死时的兜底。

用法：
    python3 -u _synth_odo_test.py
"""

import sys
import time
import unittest

from bunker_mini.protocol import MotionFeedback, OdometerFeedback

# Avoid constructing a real CAN bus: build the controller object without
# running its __init__ (the synthetic odometer logic only touches plain fields).
from bunker_mini.controller import BunkerMiniController


def _bare_controller(wheelbase_m: float = 0.5) -> BunkerMiniController:
    ctrl = object.__new__(BunkerMiniController)
    ctrl._wheelbase_m = wheelbase_m
    ctrl._synth_left_mm = 0.0
    ctrl._synth_right_mm = 0.0
    ctrl._synth_t = None
    ctrl._synth_moved = False
    ctrl._real_odo_key = None
    ctrl._real_odo_moved = False
    ctrl._real_odo_moved_t = 0.0
    ctrl._fused_left_mm = 0.0
    ctrl._fused_right_mm = 0.0
    ctrl._synth_ref_left = 0.0
    ctrl._synth_ref_right = 0.0
    ctrl._last_motion_t = None
    ctrl._latest_odometer = None
    ctrl._odometer_callbacks = []
    return ctrl


class SyntheticOdometerTest(unittest.TestCase):
    def test_real_odometer_preferred_when_alive(self) -> None:
        ctrl = _bare_controller()
        # 0x311 真实里程推进
        ctrl._latest_odometer = OdometerFeedback(1000, 1000)
        ctrl._real_odo_key = (1000, 1000)
        ctrl._real_odo_moved = True
        ctrl._real_odo_moved_t = time.monotonic()
        # 0x221 速度积分也累积了
        ctrl._synth_left_mm = 1234.0
        ctrl._synth_right_mm = 1234.0
        ctrl._synth_moved = True
        self.assertEqual(ctrl.latest_odometer, OdometerFeedback(1000, 1000))

    def test_synthetic_used_when_real_stuck_at_zero(self) -> None:
        ctrl = _bare_controller()
        # 0x311 帧到了但恒为 0（遥控模式）
        ctrl._latest_odometer = OdometerFeedback(0, 0)
        ctrl._real_odo_key = (0, 0)
        ctrl._real_odo_moved = False
        # 0x221 速度积分在累积
        ctrl._synth_left_mm = 250.0
        ctrl._synth_right_mm = 250.0
        ctrl._synth_moved = True
        self.assertEqual(ctrl.latest_odometer, OdometerFeedback(250, 250))

    def test_synthetic_used_when_real_never_arrived(self) -> None:
        ctrl = _bare_controller()
        ctrl._latest_odometer = None
        ctrl._synth_left_mm = 880.0
        ctrl._synth_right_mm = 880.0
        ctrl._synth_moved = True
        self.assertEqual(ctrl.latest_odometer, OdometerFeedback(880, 880))

    def test_none_when_no_feedback_at_all(self) -> None:
        ctrl = _bare_controller()
        self.assertIsNone(ctrl.latest_odometer)

    def test_integrate_straight_line(self) -> None:
        ctrl = _bare_controller()
        ctrl._synth_t = 0.0
        # 0.5 m/s 直线走 1 秒 → 每轮 500 mm
        for i in range(10):
            ctrl._integrate_synthetic_odometer(
                MotionFeedback(linear_velocity_m_s=0.5, angular_velocity_rad_s=0.0),
                now=0.1 * (i + 1),
            )
        self.assertAlmostEqual(ctrl._synth_left_mm, 500.0, delta=1.0)
        self.assertAlmostEqual(ctrl._synth_right_mm, 500.0, delta=1.0)
        self.assertTrue(ctrl._synth_moved)

    def test_integrate_turn_splits_wheels(self) -> None:
        ctrl = _bare_controller(wheelbase_m=0.5)
        ctrl._synth_t = 0.0
        # 原地转向：v=0, w=1.0 rad/s, wheelbase 0.5 → 左右轮 ±0.25 m/s
        for i in range(10):
            ctrl._integrate_synthetic_odometer(
                MotionFeedback(linear_velocity_m_s=0.0, angular_velocity_rad_s=1.0),
                now=0.1 * (i + 1),
            )
        # 1 秒：v_l = 0 - 1.0*0.25 = -0.25 → -250mm, v_r = +0.25 → +250mm
        self.assertAlmostEqual(ctrl._synth_left_mm, -250.0, delta=2.0)
        self.assertAlmostEqual(ctrl._synth_right_mm, 250.0, delta=2.0)

    def test_large_gap_ignored(self) -> None:
        ctrl = _bare_controller()
        ctrl._synth_t = 0.0
        # 2 秒的大空隙应被忽略（防止帧丢失注入巨大里程）
        ctrl._integrate_synthetic_odometer(
            MotionFeedback(linear_velocity_m_s=1.0, angular_velocity_rad_s=0.0),
            now=2.0,
        )
        self.assertEqual(ctrl._synth_left_mm, 0.0)
        self.assertEqual(ctrl._synth_right_mm, 0.0)

    def test_handle_message_rebases_on_real_advance(self) -> None:
        ctrl = _bare_controller()
        # 先用速度积分出 300mm 合成里程
        ctrl._synth_left_mm = 300.0
        ctrl._synth_right_mm = 300.0
        ctrl._synth_moved = True
        from bunker_mini.protocol import CanId
        from unittest.mock import MagicMock
        # 首帧 (0,0) → 不重基准（车已在动）
        msg0 = MagicMock()
        msg0.arbitration_id = CanId.ODOMETER
        msg0.data = bytes(8)
        ctrl._handle_message(msg0)
        self.assertEqual(ctrl.latest_odometer, OdometerFeedback(300, 300))
        # 真实里程推进到 400 → 重基准到 400
        msg = MagicMock()
        msg.arbitration_id = CanId.ODOMETER
        msg.data = bytes([0, 0, 1, 0x90, 0, 0, 1, 0x90])  # 400, 400
        ctrl._handle_message(msg)
        self.assertEqual(ctrl.latest_odometer, OdometerFeedback(400, 400))
        self.assertAlmostEqual(ctrl._synth_left_mm, 400.0)

    def test_handle_message_stuck_zero_keeps_synthetic(self) -> None:
        ctrl = _bare_controller()
        ctrl._synth_left_mm = 300.0
        ctrl._synth_right_mm = 300.0
        ctrl._synth_moved = True
        from bunker_mini.protocol import CanId
        from unittest.mock import MagicMock
        # 0x311 恒为 0（遥控模式），值从不变化 → 不重基准，继续用合成里程
        for _ in range(3):
            msg = MagicMock()
            msg.arbitration_id = CanId.ODOMETER
            msg.data = bytes(8)
            ctrl._handle_message(msg)
        self.assertEqual(ctrl.latest_odometer, OdometerFeedback(300, 300))

    def test_fused_continues_after_real_freezes_no_stall_window(self) -> None:
        """真实推进到 1000 后卡死 → 融合值 = 1000 + 合成增量，无 0.5s 冻结。"""
        ctrl = _bare_controller()
        import struct
        from bunker_mini.protocol import CanId
        from unittest.mock import MagicMock

        def _frame(l, r):
            m = MagicMock()
            m.arbitration_id = CanId.ODOMETER
            m.data = struct.pack(">ii", l, r)
            return m

        # 首帧 1000 → 推进到 1010（触发 moved=True）
        ctrl._handle_message(_frame(1000, 1000))
        ctrl._handle_message(_frame(1010, 1010))
        self.assertEqual(ctrl.latest_odometer, OdometerFeedback(1010, 1010))
        self.assertEqual(ctrl.odometer_source, "real")

        # 真实卡死 1.0s（超过 0.5s 阈值），合成里程继续积分 100mm
        ctrl._real_odo_moved_t = time.monotonic() - 1.0
        ctrl._synth_left_mm = 1110.0
        ctrl._synth_right_mm = 1110.0
        self.assertEqual(ctrl.latest_odometer, OdometerFeedback(1110, 1110))
        self.assertEqual(ctrl.odometer_source, "fused")

        # 合成里程再前进 → 融合值继续跟随，不冻结
        ctrl._synth_left_mm = 1230.0
        ctrl._synth_right_mm = 1230.0
        self.assertEqual(ctrl.latest_odometer, OdometerFeedback(1230, 1230))

    def test_live_odometer_fills_gap_while_real_alive(self) -> None:
        """0x311 仍存活时 live 也要带上 0x221 增量，goto 才看得到位移。"""
        ctrl = _bare_controller()
        import struct
        from bunker_mini.protocol import CanId
        from unittest.mock import MagicMock

        msg = MagicMock()
        msg.arbitration_id = CanId.ODOMETER
        msg.data = struct.pack(">ii", 1000, 1000)
        ctrl._handle_message(msg)
        msg2 = MagicMock()
        msg2.arbitration_id = CanId.ODOMETER
        msg2.data = struct.pack(">ii", 1010, 1010)
        ctrl._handle_message(msg2)
        self.assertEqual(ctrl.latest_odometer, OdometerFeedback(1010, 1010))
        self.assertEqual(ctrl.live_odometer, OdometerFeedback(1010, 1010))

        ctrl._synth_left_mm = 1060.0
        ctrl._synth_right_mm = 1060.0
        self.assertEqual(ctrl.latest_odometer, OdometerFeedback(1010, 1010))
        self.assertEqual(ctrl.live_odometer, OdometerFeedback(1060, 1060))

    def test_fused_rebases_when_real_resumes(self) -> None:
        """真实恢复推进 → 融合值回跳到真实值（rebase）。"""
        ctrl = _bare_controller()
        import struct
        from bunker_mini.protocol import CanId
        from unittest.mock import MagicMock

        msg = MagicMock()
        msg.arbitration_id = CanId.ODOMETER
        msg.data = struct.pack(">ii", 1000, 1000)
        ctrl._handle_message(msg)
        ctrl._synth_left_mm = 1200.0
        ctrl._synth_right_mm = 1200.0
        ctrl._real_odo_moved_t = time.monotonic() - 1.0

        # 真实恢复到 2000
        msg2 = MagicMock()
        msg2.arbitration_id = CanId.ODOMETER
        msg2.data = struct.pack(">ii", 2000, 2000)
        ctrl._handle_message(msg2)
        self.assertEqual(ctrl.latest_odometer, OdometerFeedback(2000, 2000))
        self.assertEqual(ctrl.odometer_source, "real")
        # 合成与基准都已 rebase 到真实值
        self.assertAlmostEqual(ctrl._synth_ref_left, 2000.0)


if __name__ == "__main__":
    suite = unittest.TestLoader().loadTestsFromTestCase(SyntheticOdometerTest)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
