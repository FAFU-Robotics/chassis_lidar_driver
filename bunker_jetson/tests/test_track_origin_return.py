"""倒放粗回 + 里程系精停，以及录制质量门控。"""

from __future__ import annotations

import threading
import time
import unittest
from unittest.mock import patch

import bunker_mini.agent as agent_mod
from bunker_mini.agent import BunkerMiniAgent, Command, EventType
from bunker_mini.navigator import Pose2D
from bunker_mini.tracker import (
    PlaybackDockConfig,
    Track,
    TrackPlayer,
    TrackRecorder,
    Waypoint,
    track_origin_quality,
    track_web_summary,
)


def _wp(t, l, r, v=0.2, w=0.0) -> Waypoint:
    return Waypoint(t=t, left_mm=l, right_mm=r, v=v, w=w)


def _good_track(**kw) -> Track:
    data = dict(
        name="r1",
        created_at="t",
        total_duration_s=1.0,
        waypoints=[_wp(0.0, 0, 0, 0.0), _wp(1.0, 200, 200)],
        odometer_source="real",
        drive_mode="kb",
        start_x=0.0,
        start_y=0.0,
        start_yaw_deg=0.0,
    )
    data.update(kw)
    return Track(**data)


class _OdoCtrl:
    def __init__(self) -> None:
        self._odo = type("O", (), {"left_wheel_mm": 0, "right_wheel_mm": 0})()
        self._motion = type("M", (), {
            "linear_velocity_m_s": 0.0, "angular_velocity_rad_s": 0.0,
        })()
        self.cmds: list[tuple[float, float]] = []
        self.odometer_source = "real"
        self.stops = 0

    @property
    def latest_odometer(self):
        return self._odo

    @property
    def latest_motion(self):
        return self._motion

    def set_odometer(self, l: int, r: int) -> None:
        self._odo = type("O", (), {"left_wheel_mm": l, "right_wheel_mm": r})()

    def set_motion(self, v: float, w: float) -> None:
        self._motion = type("M", (), {
            "linear_velocity_m_s": v, "angular_velocity_rad_s": w,
        })()

    def set_velocity(self, v: float, w: float) -> None:
        self.cmds.append((v, w))

    def set_velocity_now(self, v: float, w: float) -> None:
        self.cmds.append((v, w))

    def stop_motion(self) -> None:
        self.stops += 1
        self.cmds.append((0.0, 0.0))

    def stop(self) -> None:
        pass


def _make_agent() -> BunkerMiniAgent:
    with patch.object(agent_mod, "resolve_can_config", lambda *a, **k: ("0", "virtual")), \
            patch.object(agent_mod, "usb_socketcan_channels", lambda: []), \
            patch.object(agent_mod, "restore_tx_mode", lambda channels=None: None):
        a = BunkerMiniAgent(
            ws_url="ws://127.0.0.1:1/test",
            device_id="TEST-01",
            bind_code="TEST-BIND-xxxx",
            local_mode=True,
        )
    a._controller = _OdoCtrl()
    return a


class TestOriginQuality(unittest.TestCase):
    def test_requires_wheel_odo_and_rejects_synthetic(self):
        good = _good_track()
        q = track_origin_quality(good)
        self.assertTrue(q["ok"] and q["dock_ok"] and q["has_odo"])
        summary = track_web_summary(good)
        self.assertTrue(summary["returnOk"] and summary["dockOk"])

        no_odo = _good_track(waypoints=[_wp(0.0, 0, 0, 0.2), _wp(1.0, 0, 0, 0.2)])
        q2 = track_origin_quality(no_odo)
        self.assertFalse(q2["ok"])
        self.assertFalse(q2["has_odo"])

        synth = _good_track(odometer_source="synthetic")
        q3 = track_origin_quality(synth)
        self.assertFalse(q3["ok"])
        self.assertFalse(track_web_summary(synth)["returnOk"])

    def test_json_roundtrip_start_pose(self):
        ctrl = _OdoCtrl()
        rec = TrackRecorder(ctrl, adaptive=False, sample_interval_s=0.05)
        rec.start("p1")
        rec._rec_t0 = time.monotonic() - 0.2
        rec._waypoints = [_wp(0.0, 0, 0, 0.0), _wp(0.2, 80, 80)]
        ctrl.set_odometer(80, 80)
        ctrl.set_motion(0.0, 0.0)
        track = rec.stop()
        track.start_x, track.start_y, track.start_yaw_deg = 0.5, -0.1, 12.0
        parsed = Track.from_json(track.to_json())
        self.assertAlmostEqual(parsed.start_x, 0.5)
        self.assertAlmostEqual(parsed.start_yaw_deg, 12.0)
        self.assertEqual(parsed.odometer_source, "real")


class TestReverseNoSkip(unittest.TestCase):
    def test_reverse_guard_abort_does_not_skip(self):
        ctrl = _OdoCtrl()

        def always_block(v, w):
            return 0.0, 0.0, True

        player = TrackPlayer(
            ctrl,
            velocity_guard=always_block,
            correction=None,
            dock=PlaybackDockConfig(enabled=False),
        )
        track = _good_track(drive_mode="remote")
        done: list[bool] = []

        def play():
            done.append(player.play(track.reversed(), reverse=True, bypass_guard=False))

        t = threading.Thread(target=play, daemon=True)
        t.start()
        t.join(timeout=4.0)
        self.assertTrue(done)
        self.assertFalse(done[0])
        self.assertTrue(player._aborted_by_guard)


class TestAgentRecordAndDock(unittest.TestCase):
    def tearDown(self):
        a = getattr(self, "agent", None)
        if a is not None:
            a.stop()
            a._cleanup()

    def test_record_refuses_while_moving(self):
        self.agent = _make_agent()
        events: list[tuple] = []
        self.agent._send_event = lambda e, m: events.append((e, m))
        rec = TrackRecorder(self.agent._controller, adaptive=False)
        self.agent._recorder = rec
        self.agent._controller.set_motion(0.20, 0.0)
        t0 = time.monotonic()
        self.agent._handle_track_record(Command(action="track_record", name="busy"))
        self.assertLess(time.monotonic() - t0, 2.0)
        self.assertFalse(rec.is_recording)
        self.assertTrue(events and events[-1][0] == "fault")
        self.assertIn("还在动", events[-1][1])

    def test_record_refuses_synthetic_odometer(self):
        self.agent = _make_agent()
        events: list[tuple] = []
        self.agent._send_event = lambda e, m: events.append((e, m))
        rec = TrackRecorder(self.agent._controller, adaptive=False)
        self.agent._recorder = rec
        self.agent._controller.odometer_source = "synthetic"
        self.agent._handle_track_record(Command(action="track_record", name="rc"))
        self.assertFalse(rec.is_recording)
        self.assertTrue(events and "synthetic" in events[-1][1])

    def test_reverse_refuses_track_without_wheel_odo(self):
        self.agent = _make_agent()
        events: list[tuple] = []
        self.agent._send_event = lambda e, m: events.append((e, m))

        class _P:
            is_playing = False
            last_playback_had_odo = False
            last_playback_stalled_wps = 0
            last_playback_time_fallback = True

            def load_track(self, name):
                return _good_track(
                    waypoints=[_wp(0.0, 0, 0, 0.2), _wp(1.0, 0, 0, 0.2)],
                    odometer_source="unknown",
                )

            def stop(self):
                pass

            def play_async(self, *a, **k):
                raise AssertionError("must not play")

        self.agent._player = _P()
        self.agent._handle_track_follow(
            Command(action="track_follow", track_id="bad", reverse=True))
        self.assertTrue(events and events[-1][0] == "fault")
        self.assertIn("0x311", events[-1][1])

    def test_reverse_complete_docks_to_recorded_start(self):
        self.agent = _make_agent()
        events: list[tuple] = []
        self.agent._send_event = lambda e, m: events.append((e, m))

        class _Nav:
            def __init__(self) -> None:
                self.pose = Pose2D(0.35, 0.12, 0.2)
                self.is_navigating = False
                self.goals: list = []

            def stop(self):
                self.is_navigating = False

            def set_guard(self, guard):
                pass

            def goto(self, x, y, *, on_arrived=None, on_abort=None, speed=None,
                     waypoints=None, replanner=None, goal_yaw=None):
                self.goals.append((x, y, goal_yaw))
                self.pose = Pose2D(x, y, goal_yaw or 0.0)
                if on_arrived:
                    on_arrived()
                return True

            def feed_odometry(self, *_a):
                pass

        class _P:
            is_playing = False
            last_playback_had_odo = True
            last_playback_stalled_wps = 0
            last_playback_time_fallback = False
            played = False

            def load_track(self, name):
                return _good_track()

            def stop(self):
                pass

            def play_async(self, track, on_complete=None, reverse=False, bypass_guard=False):
                self.played = True
                assert reverse is True
                if on_complete:
                    on_complete(True)

        nav = _Nav()
        self.agent._navigator = nav
        self.agent._player = _P()
        self.agent._handle_track_follow(
            Command(action="track_follow", track_id="r1", reverse=True))
        self.assertTrue(self.agent._player.played)
        self.assertTrue(nav.goals)
        self.assertAlmostEqual(nav.goals[0][0], 0.0)
        self.assertAlmostEqual(nav.goals[0][1], 0.0)
        self.assertEqual(events[-1][0], EventType.ARRIVED)
        self.assertIn("docked at recorded start", events[-1][1])
        self.assertTrue(self.agent._last_replay and self.agent._last_replay["docked"])

    def test_reverse_incomplete_does_not_claim_back_at_start(self):
        self.agent = _make_agent()
        events: list[tuple] = []
        self.agent._send_event = lambda e, m: events.append((e, m))

        class _P:
            is_playing = False
            last_playback_had_odo = True
            last_playback_stalled_wps = 4
            last_playback_time_fallback = False

            def load_track(self, name):
                return _good_track()

            def stop(self):
                pass

            def play_async(self, track, on_complete=None, reverse=False, bypass_guard=False):
                if on_complete:
                    on_complete(False)

        class _Nav:
            pose = Pose2D(2.0, 0.0, 0.0)
            is_navigating = False

            def stop(self):
                pass

            def goto(self, *a, **k):
                raise AssertionError("incomplete reverse must not dock across the room")

        self.agent._navigator = _Nav()
        self.agent._player = _P()
        self.agent._handle_track_follow(
            Command(action="track_follow", track_id="r1", reverse=True))
        self.assertEqual(events[-1][0], EventType.ARRIVED)
        self.assertIn("interrupted", events[-1][1])
        self.assertNotIn("back at start", events[-1][1])
        self.assertFalse(self.agent._last_replay["complete"])
        self.assertEqual(self.agent._last_replay["stalledWps"], 4)


if __name__ == "__main__":
    unittest.main()
