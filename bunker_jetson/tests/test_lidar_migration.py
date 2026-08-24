"""Tests for capabilities migrated from the teammate's robosense_airy repo.

覆盖：
  A. DIFOP 出厂标定解析（RPM / FOV / 安装模式 / 垂直水平角）及其应用
  B. PCAP 离线回放（纯标准库解析 + PcapReplaySource 回放）
  C. 自扫硬件过滤（安装支架区域剔除）
  D. 履带外扩侧向间隙 + 低矮障碍（碎石）计数
  E. 底盘诊断 hook（STANDBY 自动重使能 + 指令/实测轮速对比）
  F. 安装外参变换（离地高度 + 俯仰）
"""

import math
import struct
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from bunker_mini.lidar import (
    DIFOP_MAGIC,
    LidarPoint,
    MSOP_PACKET_SIZE,
    SelfMaskConfig,
    filter_self_hardware,
    parse_difop_packet,
    parse_msop_packet,
    transform_point_cloud,
)
from bunker_mini.obstacle import count_low_obstacles, track_side_clearance
from bunker_mini.pcap import PcapReplaySource, load_msop_packets
from bunker_mini.protocol import ControlMode


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------

def _make_msop_packet(*, azimuth_raw: int = 139, distance_raw: int = 269) -> bytes:
    """Valid 1248-byte Airy MSOP packet (distance 1.345 m, azimuth 1.39°)."""
    data = bytearray(MSOP_PACKET_SIZE)
    struct.pack_into(">I", data, 0, 0x55AA055A)
    struct.pack_into(">I", data, 12, 1)
    data[31] = 0x31
    for block in range(8):
        off = 42 + block * 148
        struct.pack_into(">H", data, off, 0xFFEE)
        struct.pack_into(">H", data, off + 2, azimuth_raw)
        base = off + 4
        for ch in range(48):
            struct.pack_into(">HB", data, base + ch * 3, distance_raw, 0x6E)
    return bytes(data)


def _make_difop_packet(
    *,
    rpm: int = 600,
    fov_start: int = 0,
    fov_end: int = 36000,
    install_mode: int = 0,
    vert_deg: tuple[float, ...] | None = None,
    horiz_deg: tuple[float, ...] | None = None,
) -> bytes:
    data = bytearray(1248)
    data[:8] = DIFOP_MAGIC
    struct.pack_into(">H", data, 8, rpm)
    struct.pack_into(">H", data, 32, fov_start)
    struct.pack_into(">H", data, 34, fov_end)
    data[289] = install_mode

    def _table(offset: int, angles: tuple[float, ...]) -> None:
        for i, a in enumerate(angles):
            sign = 1 if a < 0 else 0
            val = int(round(abs(a) * 100))
            data[offset + i * 3] = sign
            struct.pack_into(">H", data, offset + i * 3 + 1, val)

    if vert_deg is not None:
        _table(468, vert_deg)
    if horiz_deg is not None:
        _table(756, horiz_deg)
    return bytes(data)


def _mk_point(x: float, y: float, z: float,
              az: float | None = None, vert: float | None = None) -> LidarPoint:
    if az is None:
        az = math.degrees(math.atan2(x, y)) % 360.0
    if vert is None:
        vert = math.degrees(math.atan2(z, math.hypot(x, y)))
    return LidarPoint(
        azimuth_deg=az, vertical_deg=vert,
        distance_m=math.hypot(x, y, z), reflectivity=100, channel=1,
        x=x, y=y, z=z,
    )


def _write_pcap(path: Path, packets: list[bytes], *, port: int = 6699) -> None:
    """Write a classic PCAP file wrapping MSOP payloads in fake Eth/IPv4/UDP."""
    frames = []
    for pkt in packets:
        udp = struct.pack("!HHHH", 12345, port, 8 + len(pkt), 0) + pkt
        ip_total = 20 + len(udp)
        ip = bytearray(20)
        ip[0] = 0x45
        struct.pack_into("!H", ip, 2, ip_total)
        ip[8] = 64  # TTL
        ip[9] = 17  # UDP
        eth = bytes(12) + struct.pack("!H", 0x0800)
        frames.append(eth + bytes(ip) + udp)
    with path.open("wb") as f:
        f.write(struct.pack("<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0, 65535, 1))
        for frame in frames:
            f.write(struct.pack("<IIII", 1, 0, len(frame), len(frame)))
            f.write(frame)


# --------------------------------------------------------------------------
# A. DIFOP 标定
# --------------------------------------------------------------------------

class TestDifop:

    def test_parse_full(self):
        vert = tuple(-0.07 + i * 0.88 for i in range(96))  # 近水平 → 88°
        pkt = _make_difop_packet(rpm=600, fov_start=0, fov_end=18000,
                                 install_mode=1, vert_deg=vert)
        cal = parse_difop_packet(pkt)
        assert cal is not None
        assert cal.rps == pytest.approx(10.0)
        assert cal.fov_start_deg == 0.0
        assert cal.fov_end_deg == 180.0
        assert cal.install_mode == 1
        assert cal.vertical_angles_deg is not None
        assert len(cal.vertical_angles_deg) == 96
        assert cal.vertical_angles_deg[0] == pytest.approx(-0.07, abs=1e-3)

    def test_parse_rejects_bad_magic(self):
        data = bytearray(1248)
        data[:8] = bytes([0xFF] * 8)
        assert parse_difop_packet(bytes(data)) is None

    def test_parse_rejects_missing_calibration(self):
        # 通道标定 sign=0xFF → 该通道无标定 → 整表无效（保持手册默认表）
        pkt = _make_difop_packet()
        data = bytearray(pkt)
        data[468] = 0xFF
        cal = parse_difop_packet(bytes(data))
        assert cal is not None
        assert cal.vertical_angles_deg is None

    def test_difop_overrides_vertical_angles(self):
        # 构造 96 通道垂直角全部 30°：> max_vertical_deg(15) → 不进扇区图
        vert = tuple([30.0] * 96)
        pkt = _make_difop_packet(vert_deg=vert)
        cal = parse_difop_packet(pkt)
        assert cal is not None and cal.vertical_angles_deg is not None

        msop = _make_msop_packet(azimuth_raw=0)
        frame = parse_msop_packet(
            msop, max_vertical_deg=15.0, vertical_angles_deg=cal.vertical_angles_deg)
        # 所有通道垂直角 30° > 15° → 障碍扇区图为空
        near = frame.obstacle_sectors.min_distance()
        assert near is None


# --------------------------------------------------------------------------
# B. PCAP 离线回放
# --------------------------------------------------------------------------

class TestPcapReplay:

    def test_load_msop_packets_binary(self, tmp_path):
        pkts = [_make_msop_packet(azimuth_raw=a) for a in (0, 90, 180, 270)]
        _write_pcap(tmp_path / "t.pcap", pkts)
        loaded = load_msop_packets(tmp_path / "t.pcap", backend="binary")
        assert len(loaded) == 4
        assert all(len(p) == MSOP_PACKET_SIZE for _, p in loaded)

    def test_load_missing_file_raises(self, tmp_path):
        with pytest.raises(Exception):
            load_msop_packets(tmp_path / "nope.pcap")

    def test_replay_source_produces_frames(self, tmp_path):
        pkts = [_make_msop_packet(azimuth_raw=a) for a in
                range(0, 3600, 400)]
        _write_pcap(tmp_path / "t.pcap", pkts)
        src = PcapReplaySource(tmp_path / "t.pcap", speed=100.0, loop=False)
        src.start()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and src.frame_count < 2:
            time.sleep(0.05)
        src.stop()
        assert src.frame_count >= 2
        assert src.packet_count >= 2
        # 累积扇区图应能回答前方距离查询
        d = src.nearest_in_range(0.0, 60.0)
        assert d is not None and d <= 2.0


# --------------------------------------------------------------------------
# C. 自扫硬件过滤
# --------------------------------------------------------------------------

class TestSelfHardwareMask:

    def test_removes_bracket_points(self):
        inside = _mk_point(x=0.15, y=0.30, z=0.28)      # 支架区
        outside = _mk_point(x=0.15, y=1.0, z=0.28)      # 远处，保留
        pts = [inside, outside]
        out = filter_self_hardware(pts)
        assert out == [outside]

    def test_disabled_passthrough(self):
        pts = [_mk_point(x=0.15, y=0.30, z=0.28)]
        cfg = SelfMaskConfig(enabled=False)
        assert filter_self_hardware(pts, cfg) == pts


# --------------------------------------------------------------------------
# D. 履带外扩 + 低矮障碍
# --------------------------------------------------------------------------

class TestTracksAndLowObstacles:

    def test_track_side_clearance(self):
        left_near = _mk_point(x=-0.25, y=0.15, z=0.05)
        right_far = _mk_point(x=0.25, y=0.40, z=0.05)
        center = _mk_point(x=0.0, y=0.30, z=0.05)      # 车身内，忽略
        left, right = track_side_clearance([left_near, right_far, center])
        assert left == pytest.approx(0.15)
        assert right == pytest.approx(0.40)

    def test_low_obstacle_count(self):
        gravel = _mk_point(x=0.1, y=0.8, z=0.06)        # 低矮碎石
        tall = _mk_point(x=0.0, y=0.9, z=0.30)          # 高障碍，不算低矮
        count, nearest = count_low_obstacles([gravel, tall])
        assert count == 1
        assert nearest == pytest.approx(0.8)

    def test_guard_exposes_track_clearance(self):
        from bunker_mini.obstacle import ObstacleGuard
        frame = SimpleNamespace(points=[
            _mk_point(x=-0.25, y=0.10, z=0.05),
            _mk_point(x=0.25, y=0.40, z=0.05),
        ])
        lidar = SimpleNamespace(is_receiving=True, latest_frame=frame)
        guard = ObstacleGuard(lidar)  # type: ignore[arg-type]
        left, right = guard.track_side_clearance()
        assert left == pytest.approx(0.10)
        assert right == pytest.approx(0.40)
        assert guard.track_scrape_risk(gap_m=0.12)
        assert not guard.track_scrape_risk(gap_m=0.05)
        # 两个点 z=0.05 均落在低矮障碍窗口内
        count, nearest = guard.front_low_obstacles()
        assert count == 2
        assert nearest == pytest.approx(0.10)


# --------------------------------------------------------------------------
# E. 底盘诊断 hook
# --------------------------------------------------------------------------

class TestChassisDiagnostics:

    def _make_agent(self, mode):
        from bunker_mini.agent import BunkerMiniAgent
        agent = object.__new__(BunkerMiniAgent)
        agent._controller = SimpleNamespace(
            latest_status=SimpleNamespace(control_mode=mode),
            latest_motion=SimpleNamespace(linear_velocity_m_s=0.0,
                                          angular_velocity_rad_s=0.0),
            enable_can_control=lambda: None,
            set_velocity=lambda v, w: None,
        )
        agent._last_reenable_at = 0.0
        agent._last_zero_warn_at = 0.0
        return agent

    def test_standby_reenable(self, caplog):
        agent = self._make_agent(ControlMode.STANDBY)
        calls = []
        agent._controller.enable_can_control = lambda: calls.append(1)
        with caplog.at_level("WARNING"):
            agent._drive(0.2, 0.0)
        assert calls == [1]
        assert any("STANDBY" in r.message for r in caplog.records)

    def test_can_command_no_reenable(self, caplog):
        agent = self._make_agent(ControlMode.CAN_COMMAND)
        calls = []
        agent._controller.enable_can_control = lambda: calls.append(1)
        with caplog.at_level("WARNING"):
            agent._drive(0.2, 0.0)
        assert calls == []

    def test_zero_measured_warns(self, caplog):
        agent = self._make_agent(ControlMode.CAN_COMMAND)
        with caplog.at_level("WARNING"):
            agent._drive(0.2, 0.0)
        assert any("实测轮速" in r.message for r in caplog.records)

    def test_moving_chassis_no_warn(self, caplog):
        agent = self._make_agent(ControlMode.CAN_COMMAND)
        agent._controller.latest_motion = SimpleNamespace(
            linear_velocity_m_s=0.2, angular_velocity_rad_s=0.0)
        with caplog.at_level("WARNING"):
            agent._drive(0.2, 0.0)
        assert not any("实测轮速" in r.message for r in caplog.records)


# --------------------------------------------------------------------------
# F. 安装外参变换
# --------------------------------------------------------------------------

class TestMountExtrinsics:

    def test_height_shift(self):
        pts = [_mk_point(0.0, 1.0, 0.0)]
        out = transform_point_cloud(pts, lidar_height_m=0.5)
        assert out[0].z == pytest.approx(0.5)
        assert out[0].y == pytest.approx(1.0)

    def test_pitch_tilts_forward(self):
        # 俯仰 90°：正前（y=1,z=0）→ 正上（z=1）
        pts = [_mk_point(0.0, 1.0, 0.0)]
        out = transform_point_cloud(pts, pitch_deg=90.0)
        assert out[0].z == pytest.approx(1.0, abs=1e-3)
        assert out[0].y == pytest.approx(0.0, abs=1e-3)

    def test_roll_tilts_sideways(self):
        # 横滚 90°（正 = 右侧下沉）：正右（x=1,z=0）→ 正下（z=-1）
        pts = [_mk_point(1.0, 0.0, 0.0)]
        out = transform_point_cloud(pts, roll_deg=90.0)
        assert out[0].z == pytest.approx(-1.0, abs=1e-3)
        assert out[0].x == pytest.approx(0.0, abs=1e-3)

    def test_noop_returns_same_points(self):
        pts = [_mk_point(0.0, 1.0, 0.1)]
        assert transform_point_cloud(pts) is pts
