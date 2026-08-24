"""Offline PCAP replay for the RoboSense Airy LiDAR (B 迁移).

组员代码提供 ``pcap_reader.py`` + ``pcap_replay.py`` 的离线回放能力：
用网卡抓包得到的 ``.pcap`` 文件替代实时 UDP，无需雷达硬件即可复现
真实点云，用于避障 / 巡游 / 目标检测的调试与回归。

本模块为**纯标准库**实现（``struct`` 直接解析 classic PCAP，零第三方
依赖，Windows / Linux / Jetson 通用），并提供与 :class:`AiryLidar`
接口兼容的 :class:`PcapReplaySource` —— agent 侧传入 ``lidar_pcap``
即可无缝替换 UDP 数据源，避障 / 地形 / 视觉链路完全复用。

用法::

    src = PcapReplaySource("airy_6x12_indoor.pcap", speed=1.0, loop=True)
    src.start()
    while not src.finished:
        d = src.nearest_in_range(0.0, 30.0)
        ...
    src.stop()
"""

from __future__ import annotations

import logging
import math
import struct
import threading
import time
from collections import deque
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

from .lidar import (
    AccumulatingSectors,
    LidarPoint,
    MSOP_MAGIC,
    MSOP_PACKET_SIZE,
    MSOP_PORT,
    OA_ACCUM_MAX_VERTICAL_DEG,
    ObstacleSectors,
    ScanFrame,
    SelfMaskConfig,
    filter_self_hardware,
    parse_msop_packet,
    point_cloud_snapshot,
    transform_point_cloud,
)
from .terrain import DEFAULT_STEP_LIMIT_M, TerrainProfile, TerrainSectorResult

logger = logging.getLogger(__name__)

MsopPacket = Tuple[float, bytes]  # (timestamp_sec, payload)
Backend = str  # "auto" | "binary" | "scapy"


class PcapError(ValueError):
    """Raised for unsupported / corrupt PCAP files."""


def load_msop_packets(
    pcap_path: str | Path,
    msop_port: int = MSOP_PORT,
    backend: Backend = "auto",
) -> List[MsopPacket]:
    """Extract all Airy MSOP packets (port 6699, 1248 B) from a PCAP file.

    ``backend="binary"`` 用标准库 ``struct`` 直接解析 classic PCAP；
    ``"scapy"`` 在有 scapy 时更快；``"auto"`` 优先 scapy、回退 binary。
    仅支持 classic PCAP（magic ``0xa1b2c3d4`` / ``0xd4c3b2a1``）。
    """
    path = Path(pcap_path)
    if not path.is_file():
        raise PcapError(f"PCAP 文件不存在: {path}")
    if backend in ("auto", "scapy"):
        try:
            import scapy.all  # noqa: F401
            return _load_scapy(path, msop_port)
        except (ImportError, Exception):  # noqa: BLE001 — fall back to binary
            pass
    return _load_binary(path, msop_port)


def _parse_udp_payload(frame: bytes, msop_port: int) -> Optional[bytes]:
    """Extract MSOP payload from an Ethernet/IPv4 frame (binary backend)."""
    if len(frame) < 14:
        return None
    eth_type = struct.unpack("!H", frame[12:14])[0]
    if eth_type != 0x0800:  # IPv4 only
        return None
    ip = frame[14:]
    if len(ip) < 20 or (ip[0] >> 4) != 4:
        return None
    ihl = (ip[0] & 0x0F) * 4
    if ip[9] != 17 or len(ip) < ihl + 8:  # UDP
        return None
    _sport, dport, ulen, _ = struct.unpack("!HHHH", ip[ihl:ihl + 8])
    if dport != msop_port:
        return None
    payload = ip[ihl + 8:ihl + ulen]
    if len(payload) != MSOP_PACKET_SIZE or payload[:4] != MSOP_MAGIC:
        return None
    return payload


def _load_binary(path: Path, msop_port: int) -> List[MsopPacket]:
    packets: List[MsopPacket] = []
    with path.open("rb") as f:
        gh = f.read(24)
        if len(gh) < 24:
            raise PcapError(f"无效 PCAP（文件过短）: {path}")
        magic = struct.unpack("<I", gh[:4])[0]
        if magic == 0xA1B2C3D4:
            endian = "<"
        elif magic == 0xD4C3B2A1:
            endian = ">"
        else:
            raise PcapError(
                f"不支持的 PCAP 格式（magic=0x{magic:08X}），仅支持 classic PCAP"
            )
        while True:
            ph = f.read(16)
            if len(ph) < 16:
                break
            ts_sec, ts_usec, incl, _orig = struct.unpack(endian + "IIII", ph)
            frame = f.read(incl)
            if len(frame) < incl:
                break
            payload = _parse_udp_payload(frame, msop_port)
            if payload is not None:
                packets.append((ts_sec + ts_usec * 1e-6, payload))
    if not packets:
        raise PcapError(
            f"PCAP 中未找到 MSOP 数据包（UDP 端口 {msop_port}）: {path}"
        )
    return packets


def _load_scapy(path: Path, msop_port: int) -> List[MsopPacket]:
    from scapy.all import UDP, rdpcap

    packets: List[MsopPacket] = []
    for pkt in rdpcap(str(path)):
        if UDP not in pkt:
            continue
        udp = pkt[UDP]
        if int(udp.dport) != msop_port:
            continue
        payload = bytes(udp.payload)
        if len(payload) != MSOP_PACKET_SIZE or payload[:4] != MSOP_MAGIC:
            continue
        packets.append((float(pkt.time), payload))
    if not packets:
        raise PcapError(
            f"PCAP 中未找到 MSOP 数据包（UDP 端口 {msop_port}）: {path}"
        )
    return packets


class PcapReplaySource:
    """Replay MSOP packets from a PCAP through the same decode pipeline.

    接口与 :class:`AiryLidar` 对齐（``latest_frame`` / ``is_receiving`` /
    ``nearest_in_range`` / ``terrain`` / ``terrain_sector`` / ``frame_count``
    / ``packet_count`` / ``start`` / ``stop``），可作 agent 的 ``lidar_pcap``
    数据源。后台线程按 PCAP 时间戳以 ``speed`` 倍速喂给解码器；``loop=True``
    时循环回放直到 ``stop()``。
    """

    NO_DATA_STALE_S: float = 3.0

    def __init__(
        self,
        pcap_path: str | Path,
        *,
        speed: float = 1.0,
        loop: bool = True,
        msop_port: int = MSOP_PORT,
        max_vertical_deg: float = 15.0,
        sector_count: int = 360,
        mount_yaw_deg: float = 0.0,
        apply_delay_compensation: bool = True,
        window_frames: int = 8,
        pitch_deg: float = 0.0,
        lidar_height_m: float = 0.0,
        self_mask=None,
        backend: Backend = "auto",
    ) -> None:
        self._pcap_path = Path(pcap_path)
        self._speed = max(float(speed), 0.001)
        self._loop = bool(loop)
        self._msop_port = msop_port
        self._max_vertical_deg = max_vertical_deg
        self._sector_count = sector_count
        self._mount_yaw_deg = mount_yaw_deg % 360.0
        self._apply_delay_compensation = apply_delay_compensation
        self._window_frames = max(1, window_frames)
        self._pitch_deg = float(pitch_deg) % 360.0
        self._lidar_height_m = float(lidar_height_m)
        self._self_mask = self_mask

        self._packets = load_msop_packets(pcap_path, msop_port=msop_port, backend=backend)
        if len(self._packets) < 2:
            raise PcapError(f"PCAP 包太少，无法回放: {self._pcap_path}")

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._latest_frame: Optional[ScanFrame] = None
        self._last_frame_at: float = 0.0
        self._frame_count: int = 0
        self._packet_count: int = 0
        self._finished = False

        self._window: deque[list[LidarPoint]] = deque(maxlen=self._window_frames)
        self._accum_sectors = AccumulatingSectors(max(72, self._sector_count))
        self._terrain = TerrainProfile()

    # -- lifecycle -------------------------------------------------------

    @property
    def duration_sec(self) -> float:
        return self._packets[-1][0] - self._packets[0][0]

    @property
    def finished(self) -> bool:
        """True 时回放已结束（loop=False 放完，或 stop() 后）。"""
        with self._lock:
            return self._finished

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        with self._lock:
            self._finished = False
        self._thread = threading.Thread(
            target=self._replay_loop, name="lidar-pcap", daemon=True)
        self._thread.start()
        logger.info("PCAP replay started: %s (%.1fs, %d pkts, %s)",
                    self._pcap_path, self.duration_sec, len(self._packets),
                    "loop" if self._loop else "single")

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        with self._lock:
            self._finished = True

    # -- accessors (AiryLidar-compatible) --------------------------------

    @property
    def latest_frame(self) -> Optional[ScanFrame]:
        with self._lock:
            return self._latest_frame

    @property
    def is_receiving(self) -> bool:
        with self._lock:
            if self._finished:
                return False
            return time.time() - self._last_frame_at < self.NO_DATA_STALE_S

    @property
    def frame_count(self) -> int:
        return self._frame_count

    @property
    def packet_count(self) -> int:
        return self._packet_count

    @property
    def bad_packet_count(self) -> int:
        return 0

    @property
    def calibration(self):
        return None

    @property
    def using_difop_calibration(self) -> bool:
        return False

    def nearest_in_range(self, center_deg: float, width_deg: float,
                         quantile: float = 0.25) -> Optional[float]:
        with self._lock:
            if not self._window:
                return None
            return self._accum_sectors.nearest_in_range(center_deg, width_deg, quantile)

    def sector_points(self) -> list[tuple[float, float]]:
        """Occupied-sector polar points [(azimuth_deg, distance_m)] (vehicle frame)."""
        with self._lock:
            if not self._window:
                return []
            return self._accum_sectors.occupied_polar()

    def point_cloud(
        self,
        max_points: int = 3000,
        max_range_m: float = 60.0,
    ) -> dict:
        """Latest-frame point cloud snapshot (mirrors ``AiryLidar.point_cloud``)."""
        with self._lock:
            frame = self._latest_frame
        return point_cloud_snapshot(frame, max_points=max_points,
                                    max_range_m=max_range_m)

    @property
    def terrain(self) -> TerrainProfile:
        with self._lock:
            return self._terrain

    def terrain_sector(self, angle_deg: float,
                       step_limit_m: float = DEFAULT_STEP_LIMIT_M) -> TerrainSectorResult:
        with self._lock:
            return self._terrain.sector(angle_deg, step_limit_m)

    # -- internal --------------------------------------------------------

    def _replay_loop(self) -> None:
        loop_idx = 0
        while not self._stop_event.is_set():
            loop_idx += 1
            prev_ts: Optional[float] = None
            for ts, pkt in self._packets:
                if self._stop_event.is_set():
                    return
                if prev_ts is not None:
                    delay = (ts - prev_ts) / self._speed
                    if delay > 0:
                        time.sleep(delay)
                prev_ts = ts
                self._feed(pkt)
            if not self._loop:
                break
        with self._lock:
            self._finished = True
        logger.info("PCAP replay finished (%d passes)", loop_idx)

    def _feed(self, data: bytes) -> None:
        if len(data) != MSOP_PACKET_SIZE or data[:4] != MSOP_MAGIC:
            return
        try:
            frame = parse_msop_packet(
                data,
                max_vertical_deg=self._max_vertical_deg,
                sector_count=self._sector_count,
                apply_delay_compensation=self._apply_delay_compensation,
            )
        except ValueError:
            return
        self._packet_count += 1

        if self._pitch_deg or self._lidar_height_m:
            frame.points = transform_point_cloud(
                frame.points, pitch_deg=self._pitch_deg,
                lidar_height_m=self._lidar_height_m,
            )
        if self._self_mask is not None and self._self_mask.enabled:
            frame.points = filter_self_hardware(frame.points, self._self_mask)
        if self._pitch_deg or self._lidar_height_m or (
                self._self_mask is not None and self._self_mask.enabled):
            new_map = ObstacleSectors(self._sector_count)
            for p in frame.points:
                hd = p.distance_m * math.cos(math.radians(p.vertical_deg))
                new_map.add(p.azimuth_deg, hd)
            frame.obstacle_sectors = new_map

        if self._mount_yaw_deg:
            old = frame.obstacle_sectors
            new_map = ObstacleSectors(self._sector_count)
            for idx, d in old.occupied_sectors():
                new_az = (idx * (360.0 / self._sector_count) + self._mount_yaw_deg) % 360.0
                new_map.add(new_az, d)
            frame.obstacle_sectors = new_map

        with self._lock:
            self._latest_frame = frame
            self._last_frame_at = time.time()
            self._frame_count += 1
            self._window.append(frame.points)
            self._rebuild_accum_locked()

    def _rebuild_accum_locked(self) -> None:
        accum = AccumulatingSectors(self._accum_sectors._sector_count)
        terrain = TerrainProfile()
        for pts in self._window:
            for p in pts:
                if abs(p.vertical_deg) > OA_ACCUM_MAX_VERTICAL_DEG:
                    continue
                az = (p.azimuth_deg + self._mount_yaw_deg) % 360.0
                hd = p.distance_m * math.cos(math.radians(p.vertical_deg))
                if hd >= 0.05:
                    accum.add(az, hd)
            terrain.add_frame(pts)
        self._accum_sectors = accum
        self._terrain = terrain


def replay_once(
    packets: Iterable[bytes],
    *,
    max_vertical_deg: float = 15.0,
    sector_count: int = 360,
) -> ScanFrame:
    """Synchronously decode a stream of raw MSOP payloads into one frame.

    供单测 / 小工具使用：忽略时间戳顺序，把收到的所有包解码并聚合为
    一帧（合并障碍扇区图）。
    """
    merged = ObstacleSectors(sector_count)
    points: list[LidarPoint] = []
    for data in packets:
        if len(data) != MSOP_PACKET_SIZE or data[:4] != MSOP_MAGIC:
            continue
        try:
            frame = parse_msop_packet(
                data, max_vertical_deg=max_vertical_deg,
                sector_count=sector_count,
            )
        except ValueError:
            continue
        points.extend(frame.points)
        for idx, d in frame.obstacle_sectors.occupied_sectors():
            merged.add(idx * (360.0 / sector_count), d)
    return ScanFrame(points=points, obstacle_sectors=merged)
