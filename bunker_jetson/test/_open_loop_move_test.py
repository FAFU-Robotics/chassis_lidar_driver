#!/usr/bin/env python3
"""开环定时 move：看门狗刷新不得再套雷达守卫。"""
from __future__ import annotations

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import bunker_mini.agent as agent_mod  # noqa: E402
from bunker_mini.agent import BunkerMiniAgent, Command  # noqa: E402


class _Ctrl:
    def __init__(self) -> None:
        self.calls: list = []
        self.stops = 0
        self.latest_status = None
        self.latest_motion = None

    def set_velocity(self, v, w):
        self.calls.append(("ramp", v, w))

    def set_velocity_now(self, v, w):
        self.calls.append(("now", v, w))

    def stop_motion(self):
        self.stops += 1

    def stop(self):
        pass


class _BlockingGuard:
    """任意速度都急停，模拟车头 0.16 m 立柱/椅腿。"""

    def guard_velocity(self, v, w):
        return 0.0, 0.0, True


class _Lidar:
    is_receiving = True


def _agent() -> BunkerMiniAgent:
    agent_mod.resolve_can_config = lambda *a, **k: ("0", "virtual")
    a = BunkerMiniAgent(
        ws_url="local://tcp",
        device_id="TEST-01",
        bind_code="",
        local_mode=True,
        enable_lidar=False,
    )
    a._controller = _Ctrl()
    a._lidar = _Lidar()
    a._guard = _BlockingGuard()
    return a


class OpenLoopMoveTest(unittest.TestCase):
    def tearDown(self) -> None:
        a = getattr(self, "agent", None)
        if a is not None:
            a.stop()

    def test_open_loop_ignores_blocking_guard_on_start(self) -> None:
        self.agent = _agent()
        self.agent._handle_command(
            Command(action="move", v=0.15, w=0.0, duration=2.0, open_loop=True)
        )
        ctrl = self.agent._controller
        self.assertEqual(ctrl.stops, 0)
        self.assertTrue(ctrl.calls)
        self.assertEqual(ctrl.calls[-1], ("now", 0.15, 0.0))
        self.assertTrue(self.agent._timed_move_active())

    def test_open_loop_refresh_does_not_reapply_guard(self) -> None:
        self.agent = _agent()
        self.agent._handle_command(
            Command(action="move", v=0.15, w=0.0, duration=2.0, open_loop=True)
        )
        ctrl = self.agent._controller
        n = len(ctrl.calls)
        stops = ctrl.stops
        self.agent._refresh_guarded_move()
        self.assertEqual(ctrl.stops, stops)
        self.assertEqual(len(ctrl.calls), n)
        self.assertEqual(ctrl.calls[-1], ("now", 0.15, 0.0))

    def test_closed_loop_refresh_still_stops(self) -> None:
        self.agent = _agent()
        pending = Command(action="move", v=0.15, w=0.0, duration=2.0, open_loop=False)
        self.agent._pending_command = pending
        self.agent._move_deadline = 1e18
        self.agent._refresh_guarded_move()
        self.assertGreaterEqual(self.agent._controller.stops, 1)


if __name__ == "__main__":
    unittest.main()
