#!/usr/bin/env python3
"""Offline smoke tests for the ported bunker_nav functionality.

No hardware required: exercises track serialization, obstacle guard
decision logic with a fake LiDAR, terrain traversability, odometry
dead-reckoning, occupancy grid updates, PCAP replay parsing and the
cloud agent's command parsing.
"""

from __future__ import annotations

import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bunker_mini.lidar import ObstacleSectors, ScanFrame, LidarPoint  # noqa: E402
from bunker_mini.obstacle import ObstacleGuard, ObstaclePolicy  # noqa: E402
from bunker_mini.terrain import TerrainProfile  # noqa: E402
from bunker_mini.tracker import Track, Waypoint, sanitize_track_name  # noqa: E402
from bunker_mini.navigator import OdometryPose, Pose2D  # noqa: E402
from bunker_mini.occupancy import OccupancyGrid  # noqa: E402
from bunker_mini.agent import Command  # noqa: E402

PASS = 0
FAIL = 0


def check(name: str, cond: bool, detail: str = "") -> None:
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  PASS  {name}")
    else:
        FAIL += 1
        print(f"  FAIL  {name}  {detail}")


# ---------------------------------------------------------------------------
# 1. Track serialization round-trip
# ---------------------------------------------------------------------------
def test_track_serialization() -> None:
    print("\n[1] Track serialization")
    t = Track(
        name="route1",
        created_at="2026-08-11T00:00:00",
        total_duration_s=3.0,
        waypoints=[
            Waypoint(t=0.0, left_mm=0, right_mm=0, v=0.1, w=0.0),
            Waypoint(t=1.0, left_mm=100, right_mm=100, v=0.1, w=0.0),
        ],
        wheelbase_m=0.5,
        total_distance_m=0.1,
    )
    restored = Track.from_json(t.to_json())
    check("to_json/from_json round trip", restored.to_json() == t.to_json())
    check("waypoint count preserved", len(restored.waypoints) == 2)
    check("reversed has same waypoint count", len(t.reversed().waypoints) == 2)
    check(
        "name sanitize strips separators",
        sanitize_track_name("a/b:c") == "a_b_c",
    )


# ---------------------------------------------------------------------------
# 2. ObstacleGuard with a fake LiDAR
# ---------------------------------------------------------------------------
class FakeLidar:
    def __init__(self) -> None:
        self._nearest = 0.5          # metres in front
        self._receiving = True
        self._points = []

    def set_nearest(self, d) -> None:
        self._nearest = d

    def set_receiving(self, on: bool) -> None:
        self._receiving = on

    @property
    def is_receiving(self) -> bool:
        return self._receiving

    @property
    def terrain_sector(self) -> None:
        return None  # no terrain awareness -> conservative behaviour

    def nearest_in_range(self, center_deg, width_deg, quantile=0.25):
        return self._nearest if self._nearest is not None else None


def test_obstacle_guard() -> None:
    print("\n[2] ObstacleGuard decision logic (fake LiDAR)")
    guard = ObstacleGuard(FakeLidar(), ObstaclePolicy())
    blocked_events = []
    guard.on_blocked(lambda d, reason: blocked_events.append((d, reason)))

    lidar = guard._lidar
    lidar.set_nearest(0.2)  # within stop_distance_m (0.3)
    # stop_confirm_frames=3 consecutive frames to confirm stop
    final = (0.3, 0.0, False)
    for _ in range(3):
        final = guard.guard_velocity(0.3, 0.0)
    v, w, blocked = final
    check("near obstacle triggers stop", blocked is True and v == 0.0 and w == 0.0)
    check("blocked callback fired", len(blocked_events) == 1)

    lidar.set_nearest(None)  # clear — 急停退出迟滞内仍慢速，但不算 blocked
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    check("clear path not blocked", blocked is False)
    check("stop-hold keeps slow after hard stop", 0.0 < v < 0.3)

    lidar.set_receiving(False)
    guard = ObstacleGuard(lidar, ObstaclePolicy(), require_sensor=True)
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    check("sensor-loss fail-safe stops", blocked is True and v == 0.0)


# ---------------------------------------------------------------------------
# 3. TerrainProfile / obstacle sectors
# ---------------------------------------------------------------------------
def test_terrain_and_sectors() -> None:
    print("\n[3] TerrainProfile & ObstacleSectors")
    sectors = ObstacleSectors(sector_count=8)
    for i in range(8):
        sectors.add(i * 45.0, 1.0)
    check("nearest_in_range wraps angles", sectors.nearest_in_range(350, 30) == 1.0)
    check("min_distance", sectors.min_distance() == 1.0)

    profile = TerrainProfile()
    # A clear flat floor in front (0°): low heights everywhere.
    profile.add_frame(
        [
            LidarPoint(x=0.1, y=0.5, z=0.0, azimuth_deg=0.0, vertical_deg=0.0,
                       distance_m=0.51, reflectivity=0, channel=1),
            LidarPoint(x=0.0, y=0.5, z=0.0, azimuth_deg=0.0, vertical_deg=0.0,
                       distance_m=0.5, reflectivity=0, channel=1),
        ]
    )
    res = profile.sector(0.0)
    check("terrain sector evaluates", res is not None)
    check("flat floor not blocked", res.blocked is False)
    check("terrain result has max_height", res.max_height_m == 0.0)


# ---------------------------------------------------------------------------
# 4. Odometry dead reckoning
# ---------------------------------------------------------------------------
def test_odometry_pose() -> None:
    print("\n[4] OdometryPose dead reckoning")
    pose = OdometryPose()
    pose.update(left_mm=0, right_mm=0)    # seed baseline
    pose.update(left_mm=100, right_mm=100)  # straight 0.1 m (advances +x)
    check(
        "forward integration",
        abs(pose.pose.x - 0.1) < 1e-6 and abs(pose.pose.y - 0.0) < 1e-6,
    )
    pose.update(left_mm=200, right_mm=200)
    check("continued integration", abs(pose.pose.x - 0.2) < 1e-6)
    p = pose.pose
    check("pose is Pose2D", isinstance(p, Pose2D))

    # Turning: right wheel moves more than left → CCW yaw (positive).
    pose = OdometryPose(wheelbase_m=0.5)
    pose.update(left_mm=0, right_mm=0)
    pose.update(left_mm=100, right_mm=200)
    check("turning produces positive yaw", pose.pose.yaw > 0)


# ---------------------------------------------------------------------------
# 5. OccupancyGrid
# ---------------------------------------------------------------------------
def test_occupancy_grid() -> None:
    print("\n[5] OccupancyGrid")
    grid = OccupancyGrid()
    grid.update(
        0.0, 0.0, 0.0,
        sectors=[(0.0, 1.0), (90.0, 2.0)],
        blocked_sectors=[(180.0, 0.8)],
    )
    stats = grid.snapshot(0.0, 0.0, 0.0)
    check("grid snapshot has fields", "occupied" in stats and "blocked" in stats)
    check("occupied cells recorded", len(stats["occupied"]) >= 1)


# ---------------------------------------------------------------------------
# 6. PCAP replay (synthetic minimal MSOP packet)
# ---------------------------------------------------------------------------
def test_pcap_replay() -> None:
    print("\n[6] PCAP replay source")
    from bunker_mini import pcap

    # Build a minimal valid pcap file with two real Ethernet/IPv4/UDP frames.
    import struct

    MSOP_PORT = 6699
    MSOP_PACKET_SIZE = 1248

    def make_msop() -> bytes:
        data = bytearray([0x55, 0xAA, 0x05, 0x5A])  # MSOP magic
        data += b"\x00" * (MSOP_PACKET_SIZE - 4)
        return bytes(data)

    def wrap_udp(payload: bytes) -> bytes:
        eth = b"\x00" * 12 + struct.pack("!H", 0x0800)  # Ethernet/IPv4
        udp_len = 8 + len(payload)
        udp = struct.pack("!HHHH", 50000, MSOP_PORT, udp_len, 0)
        ip_total = 20 + udp_len
        ip = struct.pack(
            "!BBHHHBBH4s4s",
            0x45, 0x00, ip_total, 0, 0, 64, 17, 0,
            b"\xc0\xa8\x01\xc8", b"\xff\xff\xff\xff",
        )
        return eth + ip + udp + payload

    msop = make_msop()
    frame = wrap_udp(msop)
    global_header = struct.pack("<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0, 65535, 1)
    with open("/tmp/smoke_test.pcap", "wb") as f:
        f.write(global_header)
        for ts_us in (0, 1000):
            record_header = struct.pack("<IIII", ts_us, 0, len(frame), len(frame))
            f.write(record_header)
            f.write(frame)

    packets = pcap.load_msop_packets("/tmp/smoke_test.pcap")
    check("pcap loads synthetic packet", len(packets) >= 2)

    source = pcap.PcapReplaySource("/tmp/smoke_test.pcap", loop=False, speed=1000.0)
    source.start()
    frame = None
    for _ in range(50):
        frame = source.latest_frame
        if frame is not None:
            break
        time.sleep(0.05)
    source.stop()
    check("replay source produces frame", frame is not None)


# ---------------------------------------------------------------------------
# 7. Cloud agent command parsing
# ---------------------------------------------------------------------------
def test_agent_command_parsing() -> None:
    print("\n[7] Agent Command parsing")
    cmd = Command.from_payload(
        {"action": "move", "v": "0.25", "w": "-0.1", "duration": "3"}
    )
    check("move action parsed", cmd.action == "move")
    check("v parsed as float", abs(cmd.v - 0.25) < 1e-9)
    check("w parsed as float", abs(cmd.w - (-0.1)) < 1e-9)

    cmd2 = Command.from_payload({"action": "goto", "x": 2.0, "y": 1.5})
    check("goto action parsed", cmd2.action == "goto")
    check("goto coords", cmd2.x == 2.0 and cmd2.y == 1.5)

    cmd3 = Command.from_payload({"action": "track_follow", "trackId": "route1", "reverse": True})
    check("track_follow parsed", cmd3.action == "track_follow" and cmd3.reverse is True)

    cmd4 = Command.from_payload({"action": "bogus", "v": "not-a-number"})
    check("malformed payload safe", cmd4.v == 0.0)

    cmd_kb = Command.from_payload(
        {"action": "move", "v": 0.1, "w": 0.0, "bypassGuard": True, "source": "kb"}
    )
    check("kb move 不过守卫", cmd_kb.bypass_guard is True)
    cmd_m = Command.from_payload({"action": "move", "v": 0.1, "w": 0.0})
    check("普通 m 仍过守卫", cmd_m.bypass_guard is False)


def main() -> int:
    test_track_serialization()
    test_obstacle_guard()
    test_terrain_and_sectors()
    test_odometry_pose()
    test_occupancy_grid()
    test_pcap_replay()
    test_agent_command_parsing()
    print(f"\n===== {PASS} passed, {FAIL} failed =====")
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
