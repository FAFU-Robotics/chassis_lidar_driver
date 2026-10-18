"""自动导航：合格图门闸、未定位拒绝、地图系 goto、2D 点击层。"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
_JETSON = _ROOT / "bunker_jetson"
if str(_JETSON) not in sys.path:
    sys.path.insert(0, str(_JETSON))
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from bunker_mini.click_layer import load_nav2_layer, parse_ros_map_yaml
from bunker_mini.localization import MapAlignment
from bunker_mini.qualified_maps import (
    is_banned,
    is_qualified,
    resolve_localization_prefix,
    save_selected_map_id,
)
from bunker_mini.ros_pose import yaw_from_quaternion


class QualifiedMapGateTest(unittest.TestCase):
    def test_lab_stems_are_banned(self) -> None:
        self.assertTrue(is_banned("lab"))
        self.assertTrue(is_banned("lab2.simplemap"))
        self.assertFalse(is_banned("room3"))

    def test_resolve_refuses_lab2_and_empty_list(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            (root / "qualified.json").write_text(
                json.dumps({"banned": ["lab", "lab2"], "maps": []}),
                encoding="utf-8",
            )
            for stem in ("lab", "lab2"):
                (root / f"{stem}.mm").write_text("x", encoding="utf-8")
                (root / f"{stem}.simplemap").write_text("x", encoding="utf-8")
            with self.assertRaises(ValueError) as ctx:
                resolve_localization_prefix("lab2", root)
            self.assertIn("lab", str(ctx.exception).lower())
            with self.assertRaises(ValueError) as ctx2:
                resolve_localization_prefix(None, root)
            self.assertIn("未指定", str(ctx2.exception))

    def test_resolve_accepts_listed_prefix(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            (root / "qualified.json").write_text(
                json.dumps({"maps": [{"id": "room1"}]}),
                encoding="utf-8",
            )
            (root / "room1.mm").write_text("x", encoding="utf-8")
            (root / "room1.simplemap").write_text("x", encoding="utf-8")
            prefix = resolve_localization_prefix("room1", root)
            self.assertEqual(prefix, root / "room1")
            self.assertTrue(is_qualified("room1", folder_doc(root)))


def folder_doc(root: Path) -> dict:
    from bunker_mini.qualified_maps import load_qualified
    return load_qualified(root)


class ClickLayerTest(unittest.TestCase):
    def test_parse_yaml_and_pgm_occupied(self) -> None:
        text = "\n".join((
            "image: m.pgm",
            "resolution: 0.05",
            "origin: [-1.0, -1.0, 0.0]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.25",
        ))
        meta = parse_ros_map_yaml(text)
        self.assertEqual(meta["image"], "m.pgm")
        self.assertAlmostEqual(float(meta["resolution"]), 0.05)
        self.assertEqual(meta["origin"][0], -1.0)
        with tempfile.TemporaryDirectory() as td:
            folder = Path(td)
            (folder / "m.yaml").write_text(text, encoding="utf-8")
            # 2x2：左上黑=占用，其余白=自由
            (folder / "m.pgm").write_bytes(b"P5\n2 2\n255\n" + bytes([0, 255, 255, 255]))
            layer = load_nav2_layer(folder / "m.yaml")
            self.assertTrue(layer["ready"])
            self.assertEqual(layer["occupiedCells"], 1)
            self.assertEqual(len(layer["occupied"]), 1)


class PoseMathTest(unittest.TestCase):
    def test_yaw_from_identity_quat(self) -> None:
        self.assertAlmostEqual(yaw_from_quaternion(0, 0, 0, 1), 0.0, places=6)

    def test_map_to_odom_then_goto_coords(self) -> None:
        align = MapAlignment(origin_x=1.0, origin_y=2.0, origin_yaw_deg=0.0)
        ox, oy, oyaw = align.map_to_odom(3.0, 4.0, 90.0)
        self.assertAlmostEqual(ox, 4.0)
        self.assertAlmostEqual(oy, 6.0)
        self.assertAlmostEqual(oyaw, 90.0)


class AutonavAgentTest(unittest.TestCase):
    def _agent(self, tmp: Path, pose=None):
        import bunker_mini.agent as agent_mod
        from bunker_mini.agent import AgentState, BunkerMiniAgent, Command
        from bunker_mini.navigator import Pose2D

        os.environ["BUNKER_MAPS_DIR"] = str(tmp)
        os.environ["BUNKER_TF_POSE"] = "0"
        (tmp / "qualified.json").write_text(
            json.dumps({"maps": [{"id": "room1"}]}), encoding="utf-8",
        )
        (tmp / "room1.mm").write_text("x", encoding="utf-8")
        (tmp / "room1.simplemap").write_text("x", encoding="utf-8")

        class _Ctrl:
            latest_status = None
            latest_motion = None
            latest_odometer = None
            live_odometer = None

            def set_velocity(self, v, w):
                pass

            def set_velocity_now(self, v, w):
                pass

            def stop_motion(self):
                pass

            def stop(self):
                pass

            def on_odometer(self, cb):
                pass

        class _Pose:
            def __init__(self, xyz):
                self.xyz = xyz

            def get_pose(self):
                return self.xyz

            def snapshot(self):
                return {"online": True, "hasMap": True, "locked": True, "bridge": True}

        class _Nav:
            def __init__(self):
                self.pose = Pose2D(0.0, 0.0, 0.0)
                self.is_navigating = False
                self.goals = []
                self.goal = None

            def stop(self):
                self.is_navigating = False

            def goto(self, x, y, **kwargs):
                self.goals.append((x, y, kwargs.get("goal_yaw")))
                self.goal = (x, y)
                self.is_navigating = True
                return True

            def apply_external_pose(self, x, y, yaw_deg):
                self.pose = Pose2D(x, y, yaw_deg * 3.141592653589793 / 180.0)

            def apply_yaw_correction(self, yaw_deg):
                self.pose = Pose2D(self.pose.x, self.pose.y, yaw_deg * 3.141592653589793 / 180.0)

        def _fake_resolve(channel, interface, *, allow_auto_channel=True):
            return "0", "virtual"

        orig = agent_mod.resolve_can_config
        agent_mod.resolve_can_config = _fake_resolve
        prev_maps = os.environ.get("BUNKER_MAPS_DIR")
        prev_tf = os.environ.get("BUNKER_TF_POSE")
        try:
            a = BunkerMiniAgent(
                ws_url="ws://127.0.0.1:1/test",
                device_id="TEST-01",
                bind_code="TEST-BIND-xxxx",
                enable_lidar=False,
                pose_source=pose if pose is not None else _Pose((1.0, 2.0, 0.0)),
                local_mode=True,
                autonav_yaw_match=True,
            )
        finally:
            agent_mod.resolve_can_config = orig

        def _restore() -> None:
            a.stop()
            if prev_maps is None:
                os.environ.pop("BUNKER_MAPS_DIR", None)
            else:
                os.environ["BUNKER_MAPS_DIR"] = prev_maps
            if prev_tf is None:
                os.environ.pop("BUNKER_TF_POSE", None)
            else:
                os.environ["BUNKER_TF_POSE"] = prev_tf

        self.addCleanup(_restore)
        a._controller = _Ctrl()
        a._navigator = _Nav()
        a._state = AgentState.ONLINE
        a._lidar = None
        return a, Command

    def test_autonav_goto_refuses_without_lock(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)
            class _NonePose:
                def get_pose(self):
                    return None
                def snapshot(self):
                    return {"online": False, "hasMap": False, "locked": False}
            a, Command = self._agent(tmp, pose=_NonePose())
            a._autonav_map_id = "room1"
            a._handle_command(Command(action="autonav_goto", x=3.0, y=4.0, frame="map"))
            self.assertEqual(a._navigator.goals, [])
            self.assertFalse(a._autonav_active)

    def test_autonav_goto_map_to_odom(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)
            a, Command = self._agent(tmp)
            a._autonav_map_id = "room1"
            a._localization_tick()
            self.assertEqual(a._loc_source, "external")
            a._map_alignment.origin_x = 1.0
            a._map_alignment.origin_y = 2.0
            a._handle_command(Command(action="autonav_goto", x=3.0, y=4.0, frame="map"))
            self.assertTrue(a._autonav_active)
            self.assertEqual(len(a._navigator.goals), 1)
            gx, gy, _ = a._navigator.goals[0]
            self.assertAlmostEqual(gx, 4.0)
            self.assertAlmostEqual(gy, 6.0)

    def test_task_goto_refused_while_autonav(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)
            a, Command = self._agent(tmp)
            a._autonav_map_id = "room1"
            a._localization_tick()
            a._handle_command(Command(action="autonav_goto", x=1.0, y=0.0))
            n_goals = len(a._navigator.goals)
            a._handle_command(Command(action="goto", x=9.0, y=9.0))
            self.assertEqual(len(a._navigator.goals), n_goals)

    def test_select_map_rejects_lab2(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)
            a, Command = self._agent(tmp)
            a._handle_command(Command(action="autonav_select_map", name="lab2"))
            self.assertNotEqual(a._autonav_map_id, "lab2")
            a._handle_command(Command(action="autonav_select_map", name="room1"))
            self.assertEqual(a._autonav_map_id, "room1")
            self.assertEqual(save_selected_map_id.__name__, "save_selected_map_id")

    def test_loc_source_clears_when_pose_lost(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)

            class _Flip:
                def __init__(self):
                    self.xyz = (1.0, 2.0, 0.0)

                def get_pose(self):
                    return self.xyz

                def snapshot(self):
                    locked = self.xyz is not None
                    return {
                        "online": True, "hasMap": True,
                        "locked": locked, "bridge": True,
                    }

            pose = _Flip()
            a, _Command = self._agent(tmp, pose=pose)
            a._localization_tick()
            self.assertEqual(a._loc_source, "external")
            pose.xyz = None
            a._localization_tick()
            self.assertEqual(a._loc_source, "odom")

    def test_bridge_heartbeat_is_not_loc_online(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)

            class _BridgeOnly:
                def get_pose(self):
                    return None

                def snapshot(self):
                    return {
                        "online": True, "hasMap": False,
                        "locked": False, "bridge": True,
                    }

            a, _Command = self._agent(tmp, pose=_BridgeOnly())
            a._autonav_map_id = "room1"
            st = a._autonav_status()
            self.assertFalse(st["locOnline"])
            self.assertEqual(st["stage"], "loc_offline")
            self.assertFalse(st["locReady"])

    def test_stick_clears_autonav_flag(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)
            a, Command = self._agent(tmp)
            a._autonav_map_id = "room1"
            a._handle_command(Command(action="autonav_goto", x=1.0, y=0.0))
            self.assertTrue(a._autonav_active)
            a._teleop_stick(0.0, 0.0)
            self.assertTrue(a._autonav_active)
            a._teleop_stick(0.20, 0.0)
            self.assertFalse(a._autonav_active)
            self.assertFalse(a._navigator.is_navigating)

    def test_lost_loc_aborts_autonav(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)

            class _Flip:
                def __init__(self):
                    self.xyz = (1.0, 2.0, 0.0)

                def get_pose(self):
                    return self.xyz

                def snapshot(self):
                    return {
                        "online": True, "hasMap": True,
                        "locked": self.xyz is not None, "bridge": True,
                    }

            pose = _Flip()
            a, Command = self._agent(tmp, pose=pose)
            a._autonav_map_id = "room1"
            a._handle_command(Command(action="autonav_goto", x=1.0, y=0.0, frame="map"))
            self.assertTrue(a._autonav_active)
            pose.xyz = None
            for _ in range(7):
                a._localization_tick()
            self.assertTrue(a._autonav_active)
            a._localization_tick()
            self.assertFalse(a._autonav_active)
            self.assertFalse(a._navigator.is_navigating)
            self.assertEqual(a._loc_source, "odom")

    def test_autonav_goto_rejects_lab2_name(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)
            a, Command = self._agent(tmp)
            a._autonav_map_id = "room1"
            a._handle_command(Command(action="autonav_goto", x=1.0, y=0.0, name="lab2", frame="map"))
            self.assertEqual(a._navigator.goals, [])
            self.assertFalse(a._autonav_active)


    def test_start_frame_goto_without_lock(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)

            class _NonePose:
                def get_pose(self):
                    return None
                def snapshot(self):
                    return {"online": False, "hasMap": False, "locked": False}

            a, Command = self._agent(tmp, pose=_NonePose())
            a._handle_command(Command(action="autonav_goto", x=3.0, y=4.0, frame="odom"))
            self.assertTrue(a._autonav_active)
            self.assertEqual(a._autonav_frame, "odom")
            self.assertEqual(len(a._navigator.goals), 1)
            gx, gy, _ = a._navigator.goals[0]
            self.assertAlmostEqual(gx, 3.0)
            self.assertAlmostEqual(gy, 4.0)

    def test_start_frame_survives_lost_tf(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)

            class _Flip:
                def __init__(self):
                    self.xyz = (1.0, 2.0, 0.0)

                def get_pose(self):
                    return self.xyz

                def snapshot(self):
                    return {
                        "online": True, "hasMap": True,
                        "locked": self.xyz is not None, "bridge": True,
                    }

            pose = _Flip()
            a, Command = self._agent(tmp, pose=pose)
            a._handle_command(Command(action="autonav_goto", x=1.0, y=0.0, frame="odom"))
            self.assertTrue(a._autonav_active)
            pose.xyz = None
            for _ in range(10):
                a._localization_tick()
            self.assertTrue(a._autonav_active)
            self.assertTrue(a._navigator.is_navigating)

    def test_passive_lidar_allows_open_loop_start_goto(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)
            a, Command = self._agent(tmp)
            a._lidar = type("L", (), {"is_receiving": False})()
            a._lidar_passive = True
            a._lidar_failsafe_armed = False
            a._handle_command(Command(action="autonav_goto", x=1.0, y=0.0, frame="odom"))
            self.assertTrue(a._autonav_active)

    def test_armed_lidar_without_data_rejects_goto(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)
            a, Command = self._agent(tmp)
            a._lidar = type("L", (), {"is_receiving": False})()
            a._lidar_passive = False
            a._lidar_failsafe_armed = True
            a._handle_command(Command(action="autonav_goto", x=1.0, y=0.0, frame="odom"))
            self.assertFalse(a._autonav_active)
            self.assertEqual(a._navigator.goals, [])

    def test_sketch_goto_without_lock(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)

            class _NonePose:
                def get_pose(self):
                    return None
                def snapshot(self):
                    return {"online": False, "hasMap": False, "locked": False}

            a, Command = self._agent(tmp, pose=_NonePose())
            a._handle_command(Command(action="autonav_goto", x=3.0, y=4.0, frame="sketch"))
            self.assertTrue(a._autonav_active)
            self.assertEqual(a._autonav_frame, "sketch")
            self.assertEqual(len(a._navigator.goals), 1)
            gx, gy, _ = a._navigator.goals[0]
            self.assertAlmostEqual(gx, 3.0)
            self.assertAlmostEqual(gy, 4.0)
            a2, Command2 = self._agent(tmp, pose=_NonePose())
            a2._handle_command(Command2(action="autonav_goto", x=1.5, y=-0.2, frame="occ"))
            self.assertEqual(a2._autonav_frame, "sketch")
            self.assertTrue(a2._autonav_active)

    def test_sketch_survives_lost_tf(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)

            class _Flip:
                def __init__(self):
                    self.xyz = (1.0, 2.0, 0.0)

                def get_pose(self):
                    return self.xyz

                def snapshot(self):
                    return {
                        "online": True, "hasMap": True,
                        "locked": self.xyz is not None, "bridge": True,
                    }

            pose = _Flip()
            a, Command = self._agent(tmp, pose=pose)
            a._map_alignment.origin_x = 10.0
            a._handle_command(Command(action="autonav_goto", x=1.0, y=0.0, frame="sketch"))
            self.assertTrue(a._autonav_active)
            gx, gy, _ = a._navigator.goals[0]
            self.assertAlmostEqual(gx, 1.0)
            self.assertAlmostEqual(gy, 0.0)
            pose.xyz = None
            for _ in range(10):
                a._localization_tick()
            self.assertTrue(a._autonav_active)
            self.assertTrue(a._navigator.is_navigating)
            self.assertEqual(a._autonav_frame, "sketch")

    def test_start_frame_tick_ignores_map_tf(self) -> None:
        from bunker_mini.navigator import Pose2D

        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)
            a, Command = self._agent(tmp)
            a._handle_command(Command(action="autonav_goto", x=1.0, y=0.0, frame="odom"))
            a._navigator.pose = Pose2D(0.2, 0.1, 0.0)
            a._localization_tick()
            self.assertAlmostEqual(a._navigator.pose.x, 0.2)
            self.assertAlmostEqual(a._navigator.pose.y, 0.1)
            self.assertNotEqual(a._loc_source, "external")

    def test_start_frame_yaw_match_keeps_xy(self) -> None:
        from bunker_mini.navigator import Pose2D
        from bunker_mini.scanmatch import ScanMatcher

        class _Lidar:
            is_receiving = True

            def sector_points(self):
                return [(0.0, 1.5)]

        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)
            a, Command = self._agent(tmp)
            a._autonav_yaw_match = True
            a._scan_match_blend = 1.0
            a._lidar = _Lidar()
            seen = []

            class _Matcher(ScanMatcher):
                def match(self, sectors, x, y, yaw_deg):
                    seen.append((x, y, yaw_deg))
                    return (0.40, -0.30, 8.0)

                def observe(self, sectors, x, y, yaw_deg):
                    return None

            a._scan_matcher = _Matcher()
            a._scan_match_last_yaw = None
            a._navigator.pose = Pose2D(1.25, -0.40, 0.0)
            a._handle_command(Command(action="autonav_goto", x=2.0, y=0.0, frame="odom"))
            a._localization_tick()
            self.assertTrue(seen)
            self.assertAlmostEqual(a._navigator.pose.x, 1.25)
            self.assertAlmostEqual(a._navigator.pose.y, -0.40)
            self.assertAlmostEqual(a._navigator.pose.yaw_deg, 8.0, places=2)
            self.assertEqual(a._loc_source, "yawmatch")
            self.assertEqual(a._loc_last_corr["dx"], 0.0)
            self.assertEqual(a._loc_last_corr["dy"], 0.0)

    def test_start_frame_yaw_match_can_disable(self) -> None:
        from bunker_mini.navigator import Pose2D
        from bunker_mini.scanmatch import ScanMatcher

        class _Lidar:
            is_receiving = True

            def sector_points(self):
                return [(0.0, 1.5)]

        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)
            a, Command = self._agent(tmp)
            a._autonav_yaw_match = False
            a._scan_match_blend = 1.0
            a._lidar = _Lidar()
            called = []

            class _Matcher(ScanMatcher):
                def match(self, sectors, x, y, yaw_deg):
                    called.append(True)
                    return (0.40, -0.30, 8.0)

                def observe(self, sectors, x, y, yaw_deg):
                    return None

            a._scan_matcher = _Matcher()
            a._navigator.pose = Pose2D(0.5, 0.0, 0.0)
            a._handle_command(Command(action="autonav_goto", x=1.0, y=0.0, frame="sketch"))
            a._localization_tick()
            self.assertFalse(called)
            self.assertAlmostEqual(a._navigator.pose.x, 0.5)
            self.assertAlmostEqual(a._navigator.pose.yaw, 0.0)

    def test_passive_lidar_full_match_does_not_shift_xy(self) -> None:
        """建图时 --no-lidar 订 /rslidar_points：空闲全匹配不得拧 XY。"""
        from bunker_mini.navigator import Pose2D
        from bunker_mini.scanmatch import ScanMatcher

        class _Lidar:
            is_receiving = True

            def sector_points(self):
                return [(0.0, 1.5)]

        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)
            a, _Command = self._agent(tmp)
            a._lidar_passive = True
            a._scan_match_blend = 1.0
            a._lidar = _Lidar()

            class _Matcher(ScanMatcher):
                def match(self, sectors, x, y, yaw_deg):
                    return (0.40, -0.30, 8.0)

                def observe(self, sectors, x, y, yaw_deg):
                    return None

            a._scan_matcher = _Matcher()
            a._scan_match_last_yaw = None
            a._navigator.pose = Pose2D(1.0, 0.2, 0.0)
            a._scan_match_tick()
            self.assertAlmostEqual(a._navigator.pose.x, 1.0)
            self.assertAlmostEqual(a._navigator.pose.y, 0.2)
            self.assertAlmostEqual(a._navigator.pose.yaw, 0.0)
            a._scan_match_tick(yaw_only=True)
            self.assertAlmostEqual(a._navigator.pose.x, 1.0)
            self.assertAlmostEqual(a._navigator.pose.y, 0.2)
            self.assertAlmostEqual(a._navigator.pose.yaw_deg, 8.0, places=2)

    def test_autonav_status_reports_yaw_match(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)
            a, _Command = self._agent(tmp)
            st = a._autonav_status()
            self.assertIn("yawMatch", st)
            self.assertTrue(st["yawMatch"])

    def test_autonav_map_keeps_odom_occupied(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            tmp = Path(td)
            a, Command = self._agent(tmp)
            a._autonav_map_id = "room1"
            a._localization_tick()
            a._map_alignment.origin_x = 10.0
            a._map_alignment.origin_y = 0.0
            captured: dict = {}

            def _cap(event, data, *, log=False):
                captured["event"] = event
                captured["data"] = data

            a._send_event_data = _cap

            class _Grid:
                def snapshot(self, x, y, yaw_deg, cap=400, include_free=False):
                    data = {"occupied": [[1.2, 0.4]], "blocked": [[0.8, -0.2]]}
                    if include_free:
                        data["free"] = [[0.2, 0.0]]
                    return data

            a._occ_grid = _Grid()
            a._handle_command(Command(action="autonav_map"))
            self.assertEqual(captured.get("event"), "autonav_map")
            data = captured["data"]
            self.assertEqual(data["odomOccupied"], [[1.2, 0.4]])
            self.assertEqual(data["odomBlocked"], [[0.8, -0.2]])
            self.assertEqual(data["odomFree"], [[0.2, 0.0]])
            self.assertIn("cloudXY", data)
            self.assertIn("vehicle", data)
            self.assertAlmostEqual(data["vehicle"]["lengthM"], 0.69)
            self.assertAlmostEqual(data["vehicle"]["widthM"], 0.57)
            self.assertIn("sketchOccupied", data)
            self.assertAlmostEqual(data["sketchOccupied"][0][0], 1.2 - 10.0)


class TfPoseSourceTest(unittest.TestCase):
    def test_stale_stamp_is_not_a_lock(self) -> None:
        import time
        from bunker_mini.ros_pose import TfPoseSource

        src = TfPoseSource(
            sock_path="/tmp/bunker_tf_pose_unittest.sock",
            spawn_bridge=False,
        )
        src._started = True
        src._last_at = time.monotonic()
        src._last = {
            "ok": True,
            "x": 1.0,
            "y": 2.0,
            "yawDeg": 0.0,
            "stampAgeS": 5.0,
            "hasMap": True,
        }
        self.assertIsNone(src.get_pose())
        self.assertFalse(src.snapshot()["hasMap"])
        src._last["stampAgeS"] = 0.05
        pose = src.get_pose()
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(pose[0], 1.0)
        self.assertTrue(src.snapshot()["hasMap"])
        self.assertTrue(src.snapshot()["locked"])


class PageScaffoldTest(unittest.TestCase):
    def test_html_has_autonav_route(self) -> None:
        html = (Path(__file__).resolve().parents[2] / "teleop_web.html").read_text(
            encoding="utf-8",
        )
        self.assertIn("#/autonav", html)
        self.assertIn("自动导航", html)
        self.assertIn("autonav_goto", html)
        self.assertIn("locReady", html)
        self.assertIn("出发系", html)
        self.assertIn('value="odom"', html)
        self.assertIn('value="sketch"', html)
        self.assertIn("现场图", html)
        self.assertIn("碰撞框", html)
        self.assertIn("车头", html)
        self.assertIn("短距里程", html)
        self.assertIn("只修航向", html)
        self.assertIn("放大", html)
        self.assertIn("缩小", html)
        self.assertIn("复位视野", html)
        self.assertIn("auto_zoom_in", html)
        self.assertIn("auto_zoom_reset", html)
        self.assertIn("autoCam", html)
        self.assertIn("autoZoomBy", html)
        self.assertIn("autoView", html)
        self.assertIn('addEventListener("wheel"', html)
        self.assertIn("autoCanvasWorld", html)
        zoom_js = html[html.find("function autoZoomBy"):html.find("function autoCanvasLocal")]
        self.assertIn("AUTO_ZOOM_MAX", zoom_js)
        self.assertIn("const AUTO_ZOOM_MAX = 3;", html)
        self.assertIn("autoCam.user = true", zoom_js)

    def test_autonav_zoom_stays_local(self) -> None:
        """放缩按钮不得落到 handleAct 末尾的 sendCmd(act)，否则会发给工控机。"""
        html = (Path(__file__).resolve().parents[2] / "teleop_web.html").read_text(
            encoding="utf-8",
        )
        start = html.find("function handleAct(act)")
        self.assertGreater(start, 0)
        body = html[start:html.find("function renderTracks", start)]
        self.assertIn('act === "auto_zoom_in"', body)
        self.assertIn("autoZoomBy", body)
        self.assertIn("autoCamReset", body)
        send_idx = body.rfind("sendCmd(act")
        zoom_idx = body.find('act === "auto_zoom_in"')
        self.assertGreater(send_idx, zoom_idx)
        self.assertIn("return", body[zoom_idx:zoom_idx + 80])

    def test_wasd_keydown_listener_not_orphaned(self) -> None:
        """自动导航改 HTML 时不得把 WASD 的 keydown 从监听器里摘出去（否则整页 JS 解析失败、灯一直未连接）。"""
        html = (Path(__file__).resolve().parents[2] / "teleop_web.html").read_text(
            encoding="utf-8",
        )
        idx = html.find("if (e.repeat)")
        self.assertGreater(idx, 0)
        prefix = html[max(0, idx - 180):idx]
        self.assertIn('addEventListener("keydown"', prefix)
        self.assertIn("connect();", html)

    def test_loc_script_does_not_default_lab2(self) -> None:
        sh = (Path(__file__).resolve().parents[2] / "maps" / "start_mola_localization.sh").read_text(
            encoding="utf-8",
        )
        self.assertIn("resolve_loc_map.py", sh)
        self.assertNotIn('PREFIX="$MAP_DIR/lab2"', sh)
        self.assertNotIn('PREFIX="$MAP_DIR/lab"', sh)


if __name__ == "__main__":
    unittest.main()
