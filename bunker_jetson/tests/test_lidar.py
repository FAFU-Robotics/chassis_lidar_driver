"""Tests for the RoboSense Airy LiDAR driver, obstacle guard and pose estimate."""

import math
import struct
import threading
from collections import deque

import pytest

from bunker_mini.lidar import (
    AccumulatingSectors,
    AiryLidar,
    CHANNEL_VERTICAL_DEG,
    MSOP_PACKET_SIZE,
    ObstacleSectors,
    parse_msop_packet,
)
from bunker_mini.navigator import OdometryPose
from bunker_mini.obstacle import ObstacleGuard, ObstaclePolicy
from bunker_mini.terrain import TerrainProfile


def _make_msop_packet(*, azimuth_raw: int = 139, distance_raw: int = 269,
                      reflectivity: int = 0x6E) -> bytes:
    """Build a valid 1248-byte MSOP packet.

    Uses the manual's worked examples: distance 0x010D=269 → 1.345 m,
    azimuth 0x008B=139 → 1.39°, reflectivity 0x6E=110.
    """
    data = bytearray(MSOP_PACKET_SIZE)
    struct.pack_into(">I", data, 0, 0x55AA055A)   # pkt_head
    struct.pack_into(">I", data, 12, 1)           # pktcnt
    data[31] = 0x31                               # lidar_type: airy
    data[32] = 0x02                               # lidar_model: 96-line
    for block in range(8):
        off = 42 + block * 148
        struct.pack_into(">H", data, off, 0xFFEE)   # data block flag (手册 0xffee)
        struct.pack_into(">H", data, off + 2, azimuth_raw)
        base = off + 4
        for ch in range(48):
            struct.pack_into(">HB", data, base + ch * 3, distance_raw, reflectivity)
    return bytes(data)


def test_parse_packet_point_count() -> None:
    frame = parse_msop_packet(_make_msop_packet())
    assert len(frame.points) == 8 * 48  # 8 blocks × 48 channels


def test_parse_distance_and_azimuth_examples() -> None:
    frame = parse_msop_packet(_make_msop_packet())
    # 手册示例：距离 269×0.5cm = 1.345 m；角度 139×0.01° = 1.39°（通道1 延时 0）
    p = next(pt for pt in frame.points if pt.channel == 1)
    assert p.distance_m == pytest.approx(1.345)
    assert p.azimuth_deg == pytest.approx(1.39)


def test_parse_channel_vertical_angles() -> None:
    frame = parse_msop_packet(_make_msop_packet())
    # 一包含 4 个 Azimuth 列，每列覆盖全部 96 通道 → 384 点
    assert len(frame.points) == 4 * len(CHANNEL_VERTICAL_DEG)
    by_channel = {}
    for pt in frame.points:
        by_channel.setdefault(pt.channel, pt)
    assert len(by_channel) == len(CHANNEL_VERTICAL_DEG)
    for ch, pt in by_channel.items():
        assert pt.vertical_deg == pytest.approx(CHANNEL_VERTICAL_DEG[ch - 1])


def test_obstacle_sectors_keep_horizontal_channels_only() -> None:
    frame = parse_msop_packet(_make_msop_packet(), max_vertical_deg=15.0)
    # 只有垂直角 ≤ 15° 的通道进扇区图；最近点即最大允许垂直角（14.92°）
    # 通道的水平投影距离
    max_vert = max(v for v in CHANNEL_VERTICAL_DEG if v <= 15.0)
    expected = 1.345 * math.cos(math.radians(max_vert))
    assert frame.obstacle_sectors.min_distance() == pytest.approx(expected, abs=0.005)


def test_nearest_in_range_wraps_angle() -> None:
    s = ObstacleSectors(360)
    s.add(359.0, 1.0)
    s.add(5.0, 2.0)
    # 0° 附近的 359° 应被探测到（环绕）
    assert s.nearest_in_range(0.0, 20.0) == pytest.approx(1.0)
    # 180° 处无障碍
    assert s.nearest_in_range(180.0, 20.0) is None


def test_invalid_packet_rejected() -> None:
    with pytest.raises(ValueError):
        parse_msop_packet(b"\x00" * 100)  # wrong length
    bad = bytearray(_make_msop_packet())
    bad[0] = 0x00  # corrupt header
    with pytest.raises(ValueError):
        parse_msop_packet(bytes(bad))


def test_airy_lidar_window_accumulation() -> None:
    """AiryLidar's sliding-window map rebuild yields robust queries (no socket)."""
    lidar = object.__new__(AiryLidar)
    lidar._lock = threading.Lock()
    lidar._mount_yaw_deg = 0.0
    lidar._window_frames = 8
    lidar._sector_count = 360
    lidar._window = deque(maxlen=8)
    lidar._accum_sectors = AccumulatingSectors(120)
    lidar._terrain = TerrainProfile()

    from bunker_mini.lidar import LidarPoint

    def _mk(az, vert, dist, z):
        az_r = math.radians(az)
        xy = dist * math.cos(math.radians(vert))
        return LidarPoint(azimuth_deg=az, vertical_deg=vert, distance_m=dist,
                          reflectivity=100, channel=1,
                          x=xy * math.sin(az_r), y=xy * math.cos(az_r), z=z)

    # 两帧相同的合成点云：近处地面 + 1.0 m 处 0.4 m 高岩壁
    for _ in range(2):
        pts = [_mk(0.0, -8.0, d, -0.35) for d in [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]]
        pts += [_mk(0.0, 2.0, 1.0, 0.05), _mk(0.0, 2.0, 1.1, 0.05)]
        lidar._window.append(pts)
        lidar._rebuild_accum_locked()

    d = lidar.nearest_in_range(0.0, 60.0)
    assert d is not None and d <= 1.1
    terrain = lidar.terrain_sector(0.0, 0.07)
    assert terrain.blocked
    assert terrain.obstacle_distance_m is not None
    assert terrain.obstacle_distance_m <= 1.1
    assert terrain.max_height_m >= 0.35


class _FakeLidar:
    """Minimal stand-in exposing the ObstacleGuard's expected interface."""

    def __init__(self, front: float | None):
        self._front = front
        self.is_receiving = True

    def nearest_in_range(self, _center, _width, quantile=0.25):
        return self._front


class _MutableLidar:
    """Fake with a mutable front distance, to test exit-hold transitions."""

    def __init__(self, front: float | None):
        self._front = front
        self.is_receiving = True

    def nearest_in_range(self, _center, _width, quantile=0.25):
        return self._front


def test_guard_hard_stops_inside_stop_distance() -> None:
    # 无地形感知的雷达按保守策略急停；急停需连续 stop_confirm_frames 帧确认
    guard = ObstacleGuard(_FakeLidar(0.2))
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    assert not blocked  # 第一帧：多帧确认中，慢速接近
    assert v < 0.3
    for _ in range(guard._policy.stop_confirm_frames - 1):
        v, w, blocked = guard.guard_velocity(0.3, 0.0)
    assert blocked
    assert v == 0.0 and w == 0.0


def test_guard_hold_after_stop_resists_obstacle_edge_jitter() -> None:
    """急停后障碍瞬间消失 → 保持慢速（退出迟滞），不立即放行全速。

    回归：障碍在 stop 边界抖动时，旧逻辑在 need_stop 一帧为 False 就
    立即放行，造成「急停→前进→急停」往复抖动。修复后急停后保持
    stop_hold_s 的慢速，障碍稳定消失才真正放行。
    """
    policy = ObstaclePolicy(stop_confirm_frames=2, stop_hold_s=0.5)
    lidar = _MutableLidar(0.2)  # 障碍在 stop 距离内
    guard = ObstacleGuard(lidar, policy)
    # 触发急停（连续 stop_confirm_frames 帧）
    for _ in range(policy.stop_confirm_frames):
        v, w, blocked = guard.guard_velocity(0.3, 0.0)
    assert blocked
    # 障碍瞬间消失（抖动），下一帧应保持慢速而非放行全速
    lidar._front = 2.0
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    assert not blocked
    assert v < 0.3, "急停后障碍瞬间消失应保持慢速（退出迟滞），不应立即全速放行"


def test_guard_slows_near_obstacle() -> None:
    policy = ObstaclePolicy(stop_distance_m=0.3, slow_distance_m=0.8,
                            slow_speed_factor=0.25)
    guard = ObstacleGuard(_FakeLidar(0.55), policy)
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    assert not blocked
    # 速度自适应安全距离：stop_eff=0.48 / slow_eff=1.16（制动距离建模）
    stop_d, slow_d = policy.effective_distances(0.3)
    t = (0.55 - stop_d) / (slow_d - stop_d)
    factor = policy.slow_speed_factor + (1.0 - policy.slow_speed_factor) * t
    assert v == pytest.approx(0.3 * factor)
    assert 0.0 < v < 0.3


def test_guard_passes_when_clear() -> None:
    guard = ObstacleGuard(_FakeLidar(2.0))
    v, w, blocked = guard.guard_velocity(0.3, 0.1)
    assert not blocked
    assert v == pytest.approx(0.3)
    assert w == pytest.approx(0.1)


def test_guard_no_sensor_passes_by_default() -> None:
    guard = ObstacleGuard(_FakeLidar(None))
    guard._lidar = None
    v, w, blocked = guard.guard_velocity(0.3, 0.0)
    assert not blocked
    assert v == pytest.approx(0.3)


def test_odometry_pose_forward() -> None:
    pose = OdometryPose(wheelbase_m=0.5)
    pose.update(1000, 1000)
    pose.update(1100, 1100)  # both wheels +100 mm → forward 0.1 m
    assert pose.pose.x == pytest.approx(0.1, abs=1e-6)
    assert pose.pose.y == pytest.approx(0.0, abs=1e-9)
    assert pose.pose.yaw == pytest.approx(0.0, abs=1e-9)


def test_odometry_pose_in_place_rotation() -> None:
    pose = OdometryPose(wheelbase_m=0.5)
    pose.update(1000, 1000)
    pose.update(1100, 900)  # differential: left +0.1, right -0.1
    # d_yaw = (dr - dl) / wheelbase = (-0.1 - 0.1) / 0.5 = -0.4 rad
    assert pose.pose.yaw == pytest.approx(-0.4, abs=1e-9)
    assert pose.pose.x == pytest.approx(0.0, abs=1e-9)
    assert pose.pose.y == pytest.approx(0.0, abs=1e-9)


def test_point_cloud_snapshot_empty_frame() -> None:
    from bunker_mini.lidar import ScanFrame, point_cloud_snapshot

    assert point_cloud_snapshot(None)["online"] is False
    assert point_cloud_snapshot(ScanFrame())["online"] is False


def test_point_cloud_snapshot_subsamples() -> None:
    from bunker_mini.lidar import LidarPoint, ScanFrame, point_cloud_snapshot

    pts = [
        LidarPoint(azimuth_deg=0.0, vertical_deg=0.0,
                   distance_m=1.0 + i * 0.1, reflectivity=i, channel=1,
                   x=float(i), y=float(i), z=0.3)
        for i in range(100)
    ]
    snap = point_cloud_snapshot(ScanFrame(points=pts),
                                max_points=10, max_range_m=60.0)
    assert snap["online"] is True
    assert snap["pointCount"] == 100
    assert len(snap["points"]) == 10
    assert all(len(p) == 4 for p in snap["points"])
    # 距车 0.05~60m 之外的过滤
    far = LidarPoint(azimuth_deg=0.0, vertical_deg=0.0, distance_m=99.0,
                     reflectivity=1, channel=1, x=99.0, y=0.0, z=0.3)
    snap2 = point_cloud_snapshot(ScanFrame(points=[far]))
    assert snap2["pointCount"] == 0


if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))
