"""RoboSense Airy LiDAR driver — UDP point cloud + horizontal obstacle map.

Airy 是速腾聚创（RoboSense）的 96 线近距离 3D 激光雷达（半球视场：垂直
0~90°、水平 360°、最近 0.1 m、10 Hz），用于机器人/无人车近场环境感知。
详见《Airy 产品手册 V1.3》。

通信：UDP over Ethernet。雷达出厂默认 IP 192.168.1.200，MSOP 端口 6699
（电脑需配静态 IP 192.168.1.102）。MSOP 包 1248 bytes，结构（手册 §4.4.2）：

    42B 帧头（pkt_head=0x55AA055A, pktcnt, timestamp, lidar_type=0x31 ...）
  1184B 数据区（8 个 Data Block，每块 148B）
    └ 每个 Block：2B 标志 0xffee + 2B Azimuth(×0.01°) + 48×3B 通道数据
      （2B Distance ×0.5cm + 1B 反射强度），每包共 8×48=384 点
     6B 帧尾

本模块无第三方依赖（仅标准库 socket/struct/threading），Windows 与
Linux/Jetson 通用，输出：

  * 完整点云（水平角 / 垂直角 / 距离 / 反射强度 / xyz）
  * 360° 水平障碍扇区图（每个扇区最近障碍距离）—— 避障 / 导航的核心输入

安装注意事项：雷达 0° 不一定朝车头，可通过 ``mount_yaw_deg`` 把雷达坐标
系对齐车体坐标系（车头 = 0°，左转为正）。
"""

from __future__ import annotations

import errno
import logging
import math
import select
import socket
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Optional

from .terrain import DEFAULT_STEP_LIMIT_M, TerrainProfile, TerrainSectorResult

logger = logging.getLogger(__name__)

# 动态姿态源：返回车体当前 (roll_deg, pitch_deg)，或 None 表示未知。
# 用于在不平/颠簸路面（月球溶洞、碎石坡）上实时校正点云的水平基准，
# 避免「车体俯仰/横滚导致把坡面误判成台阶/悬崖」。真实 IMU（雷达自带的
# IMU_PORT 6688 或外部 IMU）接入后注入即可，本框架默认无姿态源时零开销。
AttitudeSource = Callable[[], Optional[tuple[float, float]]]

DEFAULT_LIDAR_IP: str = "192.168.1.200"
MSOP_PORT: int = 6699
DIFOP_PORT: int = 7788
IMU_PORT: int = 6688

MSOP_MAGIC: bytes = bytes([0x55, 0xAA, 0x05, 0x5A])
MSOP_PACKET_SIZE: int = 1248
DATA_BLOCKS_PER_PACKET: int = 8
CHANNELS_PER_BLOCK: int = 48
CHANNELS_TOTAL: int = 96

# 雷达转速 600 RPM → 10 转/s → 3600°/s → 0.0036°/µs（发射延时角度补偿）
_ROTATION_DEG_PER_US: float = 3600.0 / 1_000_000.0

# 96 通道垂直角（度），数据来自《Airy 产品手册 V1.3》表 32（各通道测距能力）。
# 通道 1 最接近水平（-0.07°），通道 96 最高（89.4°）。
CHANNEL_VERTICAL_DEG: tuple[float, ...] = (
    -0.07, 0.88, 1.81, 2.76, 3.69, 4.62, 5.54, 6.48,
    7.41, 8.34, 9.27, 10.21, 11.15, 12.09, 13.03, 13.98,
    14.92, 15.87, 16.82, 17.77, 18.72, 19.67, 20.62, 21.57,
    22.51, 23.45, 24.40, 25.33, 26.28, 27.21, 28.15, 29.08,
    30.02, 30.95, 31.88, 32.82, 33.74, 34.68, 35.62, 36.55,
    37.50, 38.43, 39.37, 40.31, 41.25, 42.21, 43.16, 44.09,
    45.05, 46.00, 46.95, 47.90, 48.85, 49.80, 50.73, 51.69,
    52.62, 53.56, 54.50, 55.45, 56.37, 57.30, 58.24, 59.18,
    60.12, 61.05, 61.99, 62.93, 63.86, 64.81, 65.76, 66.69,
    67.65, 68.60, 69.56, 70.51, 71.46, 72.42, 73.37, 74.33,
    75.29, 76.24, 77.19, 78.14, 79.07, 80.02, 80.96, 81.90,
    82.84, 83.78, 84.70, 85.64, 86.57, 87.52, 88.46, 89.40,
)

# 各通道发射延时（µs），手册表 32 按每 8 通道分组。
# 同一水平 Azimuth 内 48 通道并非同时发射，晚发射的通道测得的角度更靠前
# 的激光打在更早的方位上，需要按延时补偿水平角。
_CHANNEL_DELAY_US: tuple[float, ...] = (
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    5.712, 5.712, 5.712, 5.712, 5.712, 5.712, 5.712, 5.712,
    12.376, 12.376, 12.376, 12.376, 12.376, 12.376, 12.376, 12.376,
    19.040, 19.040, 19.040, 19.040, 19.040, 19.040, 19.040, 19.040,
    25.704, 25.704, 25.704, 25.704, 25.704, 25.704, 25.704, 25.704,
    33.320, 33.320, 33.320, 33.320, 33.320, 33.320, 33.320, 33.320,
    41.888, 41.888, 41.888, 41.888, 41.888, 41.888, 41.888, 41.888,
    50.456, 50.456, 50.456, 50.456, 50.456, 50.456, 50.456, 50.456,
    59.024, 59.024, 59.024, 59.024, 59.024, 59.024, 59.024, 59.024,
    70.448, 70.448, 70.448, 70.448, 70.448, 70.448, 70.448, 70.448,
    81.872, 81.872, 81.872, 81.872, 81.872, 81.872, 81.872, 81.872,
    93.296, 93.296, 93.296, 93.296, 93.296, 93.296, 93.296, 93.296,
)

# 单帧扇区图：接近水平的通道（通道 1~16 ≈ -0.07°~13.98°）。
DEFAULT_MAX_VERTICAL_DEG: float = 15.0
# 累积避障扇区：放宽到椅背/人躯干的仰角，但不收近乎垂直的天花板。
# 旧逻辑把 0~90° 全塞进同一张图再取 p25，远处天花/高梁会把正前方的人/椅冲掉。
OA_ACCUM_MAX_VERTICAL_DEG: float = 40.0


def _azimuth_to_index(azimuth_deg: float, sector_count: int) -> int:
    """把方位角映射到扇区下标。``int(az) % N`` 在 N≠360 时会把 120°/240° 叠到 0°。"""
    return int(math.floor((azimuth_deg % 360.0) * sector_count / 360.0)) % sector_count

# 距离为 0 或 >60 m 的点视为无效（手册：测距能力最远 60 m）
MAX_VALID_DISTANCE_M: float = 60.0

# ---------------------------------------------------------------------------
# DIFOP（端口 7788，1248 B）— 出厂标定 / 配置包
# 偏移来自 rs_driver decoder_RSAIRY.hpp 的 RSAIRYDifopPkt 布局。
# 携带 RPM、FOV、安装模式，以及每个通道的真实垂直/水平角度标定——
# 比手册硬编码表更准（每台雷达出厂标定不同），是 3D 点云几何精度的前提。
# ---------------------------------------------------------------------------
DIFOP_PACKET_SIZE: int = 1248
DIFOP_MAGIC: bytes = bytes([0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55])
DIFOP_RPM_OFFSET: int = 8
DIFOP_FOV_START_OFFSET: int = 32
DIFOP_FOV_END_OFFSET: int = 34
DIFOP_INSTALL_MODE_OFFSET: int = 289
DIFOP_VERT_ANGLE_OFFSET: int = 468     # 每通道 3B：1B 符号 + 1B>H 幅值（×0.01°）
DIFOP_HORIZ_ANGLE_OFFSET: int = 756
_ANGLE_VALID_MIN: int = -9000
_ANGLE_VALID_MAX: int = 18000


@dataclass(frozen=True)
class DifopCalibration:
    """Radar factory calibration parsed from the DIFOP packet."""

    rps: float = 10.0                      # 实测转速（转/秒）
    fov_start_deg: float = 0.0             # FOV 起始方位（度）
    fov_end_deg: float = 360.0             # FOV 结束方位（度）
    install_mode: int = 0                  # 0=正装 1=侧装 2=Airy-M（0xFF=未知）
    vertical_angles_deg: Optional[tuple[float, ...]] = None   # 每通道垂直角标定
    horizontal_angles_deg: Optional[tuple[float, ...]] = None  # 每通道水平角标定


def parse_difop_packet(data: bytes) -> Optional[DifopCalibration]:
    """Decode a 1248-byte DIFOP packet into a :class:`DifopCalibration`.

    无法识别（长度不符 / magic 不符 / 标定未写入）时返回 None，调用方保持
    手册默认表继续工作——DIFOP 只是锦上添花，绝不阻塞点云。
    """
    if len(data) != DIFOP_PACKET_SIZE or data[:8] != DIFOP_MAGIC:
        return None

    def _angle_table(offset: int) -> Optional[tuple[float, ...]]:
        angles: list[float] = []
        for i in range(CHANNELS_TOTAL):
            off = offset + i * 3
            sign = data[off]
            if sign == 0xFF:
                return None  # 该通道无标定 → 整表无效，用默认
            try:
                val = struct.unpack_from(">H", data, off + 1)[0]
            except struct.error:
                return None
            if sign != 0:
                val = -val
            if not (_ANGLE_VALID_MIN <= val < _ANGLE_VALID_MAX):
                return None
            angles.append(val / 100.0)  # ×0.01° → 度
        return tuple(angles)

    try:
        rpm = struct.unpack_from(">H", data, DIFOP_RPM_OFFSET)[0]
        fov_start = struct.unpack_from(">H", data, DIFOP_FOV_START_OFFSET)[0]
        fov_end = struct.unpack_from(">H", data, DIFOP_FOV_END_OFFSET)[0]
    except struct.error:
        return None

    vert = _angle_table(DIFOP_VERT_ANGLE_OFFSET)
    horiz = _angle_table(DIFOP_HORIZ_ANGLE_OFFSET)

    return DifopCalibration(
        rps=(rpm / 60.0) if rpm > 0 else 10.0,
        fov_start_deg=fov_start / 100.0,
        fov_end_deg=fov_end / 100.0,
        install_mode=data[DIFOP_INSTALL_MODE_OFFSET],
        vertical_angles_deg=vert,
        horizontal_angles_deg=horiz,
    )


class LidarError(RuntimeError):
    """Raised when the LiDAR cannot be initialized (e.g. UDP port busy)."""


def msop_udp_bound(port: int = MSOP_PORT) -> bool:
    """True when any process already holds UDP ``port`` (IPv4 or IPv6).

    Airy MSOP is unicast; two binds split or steal the stream. Probe
    ``/proc/net/udp{,6}`` first so a specific-IP bind (``192.168.1.102:6699``)
    is still detected. Fall back to a non-reuse bind of ``0.0.0.0:port``.
    """
    want = f"{int(port):04X}"
    for proc in ("/proc/net/udp", "/proc/net/udp6"):
        try:
            with open(proc, encoding="ascii") as fh:
                next(fh, None)
                for line in fh:
                    cols = line.split()
                    if len(cols) < 2:
                        continue
                    local = cols[1]
                    if ":" not in local:
                        continue
                    if local.rsplit(":", 1)[-1].upper() == want:
                        return True
        except OSError:
            continue
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind(("0.0.0.0", int(port)))
        return False
    except OSError as exc:
        return getattr(exc, "errno", None) == errno.EADDRINUSE
    finally:
        probe.close()


@dataclass(frozen=True)
class LidarPoint:
    """A single measured point in the LiDAR / vehicle frame."""

    azimuth_deg: float    # 水平角（补偿发射延时后，0~360）
    vertical_deg: float   # 垂直角（通道对应）
    distance_m: float     # 斜距
    reflectivity: int     # 反射强度 1~255
    channel: int          # 通道号 1~96
    x: float              # 车体坐标（水平面内，车头方向约定见 mount_yaw_deg）
    y: float
    z: float


class ObstacleSectors:
    """360° horizontal obstacle map — nearest distance per angular sector.

    以车头方向为 0°（左转为正，与底盘角速度 w>0 左转一致）。
    用于避障查询：给定方位与张角，返回该范围内最近的障碍距离。
    注意：这是「单帧」图（一帧 MSOP 只有 4 个方位 × 96 通道，360° 稀疏），
    抗噪查询请用 :class:`AccumulatingSectors`。
    """

    __slots__ = ("_sector_count", "_dist")

    def __init__(self, sector_count: int = 360) -> None:
        if sector_count <= 0:
            raise ValueError("sector_count must be > 0")
        self._sector_count = sector_count
        self._dist: list[Optional[float]] = [None] * sector_count

    def clear(self) -> None:
        for i in range(self._sector_count):
            self._dist[i] = None

    def add(self, azimuth_deg: float, distance_m: float) -> None:
        """Record a point into its sector, keeping the nearest distance."""
        idx = _azimuth_to_index(azimuth_deg, self._sector_count)
        cur = self._dist[idx]
        if cur is None or distance_m < cur:
            self._dist[idx] = distance_m

    def nearest_in_range(self, center_deg: float, width_deg: float) -> Optional[float]:
        """Nearest obstacle within ``width_deg`` centred on ``center_deg``.

        Angles are wrapped to [0, 360); a None result means no obstacle
        in that range.
        """
        half = width_deg / 2.0
        best: Optional[float] = None
        step = 360.0 / self._sector_count
        for idx, d in enumerate(self._dist):
            if d is None:
                continue
            sector_angle = (idx * step) % 360.0
            delta = abs(((sector_angle - center_deg + 180.0) % 360.0) - 180.0)
            if delta <= half and (best is None or d < best):
                best = d
        return best

    def min_distance(self) -> Optional[float]:
        best: Optional[float] = None
        for d in self._dist:
            if d is None:
                continue
            if best is None or d < best:
                best = d
        return best

    def occupied_sectors(self) -> list[tuple[int, float]]:
        """Return [(sector_index, distance_m)] for every occupied sector."""
        return [(i, d) for i, d in enumerate(self._dist) if d is not None]

    def as_text(self, step_deg: int = 10, width: int = 64) -> str:
        """Render a compact text ring of the obstacle map (for CLI tools)."""
        lines = [f"障碍扇区图（每 {step_deg}° 一个槽，距离单位 m，'.'=无障碍）"]
        row: list[str] = []
        prev_deg = -step_deg
        for deg in range(0, 360, step_deg):
            d = self.nearest_in_range(deg, step_deg)
            if d is None:
                cell = "."
            elif d >= 10:
                cell = "##"
            else:
                cell = f"{d:2.1f}"
            label = ""
            if deg == 0:
                label = "前"
            elif deg == 90:
                label = "左"
            elif deg == 180:
                label = "后"
            elif deg == 270:
                label = "右"
            row.append(f"{deg:3d}°:{cell}{label}")
        # 每行放 8 个槽，折行输出
        for i in range(0, len(row), 8):
            lines.append("  " + "  ".join(row[i:i + 8]))
        return "\n".join(lines)


@dataclass(frozen=True)
class SelfMaskConfig:
    """Self-scan hardware filter — remove mount-bracket / cable clutter (C 迁移).

    组员代码用固定外接框滤除雷达安装支架产生的固定点（这些点稳定出现在
    车体近前、左右对称，若进扇区图/地形会形成固定误检）。实车雷达在车头
    正中、略高出车顶（约 0.35 m AGL）：自扫在保险杠/前唇一带，不是旧的
    车顶支架盒子 |x|∈[0.05,0.25], y∈[0.20,0.45], z∈[0.20,0.36]。
    """

    enabled: bool = True
    x_abs_min_m: float = 0.0
    x_abs_max_m: float = 0.20
    y_min_m: float = 0.0
    y_max_m: float = 0.16
    z_min_m: float = 0.05
    z_max_m: float = 0.28


def filter_self_hardware(
    points: list[LidarPoint],
    cfg: Optional[SelfMaskConfig] = None,
) -> list[LidarPoint]:
    """Remove points inside the self-scan hardware box (mount bracket).

    ``cfg=None`` 时用默认配置（启用）；``cfg.enabled=False`` 原样返回。
    """
    if cfg is None:
        cfg = SelfMaskConfig()
    if not cfg.enabled or not points:
        return points
    out: list[LidarPoint] = []
    for p in points:
        in_box = (
            cfg.x_abs_min_m <= abs(p.x) <= cfg.x_abs_max_m
            and cfg.y_min_m <= p.y <= cfg.y_max_m
            and cfg.z_min_m <= p.z <= cfg.z_max_m
        )
        if not in_box:
            out.append(p)
    return out


@dataclass
class ScanFrame:
    """One decoded LiDAR frame (a single MSOP packet)."""

    points: list[LidarPoint] = field(default_factory=list)
    obstacle_sectors: ObstacleSectors = field(default_factory=ObstacleSectors)
    received_at: float = field(default_factory=time.time)


def point_cloud_snapshot(
    frame: Optional[ScanFrame],
    max_points: int = 3000,
    max_range_m: float = 60.0,
) -> dict:
    """Compact latest-frame point cloud for cloud-side visualization.

    返回 ``{online, pointCount, points}``，points 为
    ``[x, y, z, intensity]`` 列表（车体系：x 右 / y 前 / z 上）。
    点数超出 ``max_points`` 时按等间隔子采样，保证发送窗口覆盖整帧。
    """
    if frame is None or not frame.points:
        return {"online": False, "pointCount": 0, "points": []}
    valid = [p for p in frame.points if 0.05 <= p.distance_m <= max_range_m]
    total = len(valid)
    if total > max_points and max_points > 0:
        step = total / max_points
        valid = [valid[int(i * step)] for i in range(max_points)]
    return {
        "online": True,
        "pointCount": total,
        "points": [
            [round(p.x, 3), round(p.y, 3), round(p.z, 3), int(p.reflectivity)]
            for p in valid
        ],
    }


class AccumulatingSectors:
    """Time-windowed 360° obstacle map — robust against noise & sparsity.

    单帧只有 4 个方位角采样，360° 非常稀疏；碎石/扬尘还会产生孤立噪点。
    把最近若干帧的点累积进同一张图后，查询时对该方位范围内的所有历史
    距离取 **低分位（p25）**：真障碍会持续出现（多数值靠近它），而瞬时
    噪点被排序挤出，既补全了 360° 覆盖又抑制了毛刺。
    """

    __slots__ = ("_sector_count", "_values")

    def __init__(self, sector_count: int = 120) -> None:
        if sector_count <= 0:
            raise ValueError("sector_count must be > 0")
        self._sector_count = sector_count
        self._values: list[list[float]] = [[] for _ in range(sector_count)]

    def clear(self) -> None:
        for v in self._values:
            v.clear()

    def add(self, azimuth_deg: float, distance_m: float) -> None:
        idx = _azimuth_to_index(azimuth_deg, self._sector_count)
        self._values[idx].append(distance_m)

    def nearest_in_range(self, center_deg: float, width_deg: float,
                         quantile: float = 0.25) -> Optional[float]:
        """Nearest obstacle (m) in range, using the ``quantile``-robust value.

        ``quantile=0.25`` → p25 低分位抗噪。角度环绕到 [0, 360)。
        """
        half = width_deg / 2.0
        step = 360.0 / self._sector_count
        vals: list[float] = []
        for idx, v in enumerate(self._values):
            if not v:
                continue
            ang = (idx * step) % 360.0
            delta = abs(((ang - center_deg + 180.0) % 360.0) - 180.0)
            if delta <= half:
                vals.extend(v)
        if not vals:
            return None
        vals.sort()
        k = max(0, min(len(vals) - 1, int(len(vals) * quantile)))
        return vals[k]

    def min_distance(self) -> Optional[float]:
        best: Optional[float] = None
        for v in self._values:
            if not v:
                continue
            m = min(v)
            if best is None or m < best:
                best = m
        return best

    def occupied_polar(self, quantile: float = 0.25) -> list[tuple[float, float]]:
        """[(azimuth_deg, distance_m)] for every sector with data.

        每个扇区取 ``quantile`` 低分位距离（默认 p25，与
        :meth:`nearest_in_range` 一致），供 2D 伪地图做射线栅格化。
        """
        step = 360.0 / self._sector_count
        out: list[tuple[float, float]] = []
        for idx, v in enumerate(self._values):
            if not v:
                continue
            vals = sorted(v)
            k = max(0, min(len(vals) - 1, int(len(vals) * quantile)))
            out.append(((idx + 0.5) * step % 360.0, vals[k]))
        return out


def parse_msop_packet(
    data: bytes,
    *,
    max_vertical_deg: float = DEFAULT_MAX_VERTICAL_DEG,
    sector_count: int = 360,
    apply_delay_compensation: bool = True,
    vertical_angles_deg: Optional[tuple[float, ...]] = None,
    horizontal_angles_deg: Optional[tuple[float, ...]] = None,
) -> ScanFrame:
    """Decode a 1248-byte MSOP packet into a ScanFrame.

    ``vertical_angles_deg`` / ``horizontal_angles_deg`` 覆盖手册硬编码表：
    传入 DIFOP 解析出的真实通道标定（96 元组）可显著提高点云几何精度。
    Raises ValueError when *data* is not a valid MSOP packet (bad length or
    frame header), so callers can skip garbage UDP payloads without crashing.
    """
    if len(data) != MSOP_PACKET_SIZE:
        raise ValueError(f"MSOP packet must be {MSOP_PACKET_SIZE} bytes, got {len(data)}")
    head, = struct.unpack_from(">I", data, 0)
    if head != 0x55AA055A:
        raise ValueError(f"bad MSOP header 0x{head:08X}")

    vert_lut: tuple[float, ...] = vertical_angles_deg or CHANNEL_VERTICAL_DEG
    horiz_lut: Optional[tuple[float, ...]] = horizontal_angles_deg

    frame = ScanFrame()
    points = frame.points
    sectors = frame.obstacle_sectors

    for block in range(DATA_BLOCKS_PER_PACKET):
        off = 42 + block * 148
        flag, azimuth_raw = struct.unpack_from(">HH", data, off)
        if flag != 0xEEFF and flag != 0xFFEE:  # 手册写 0xffee，兼容大小端书写差异
            continue
        azimuth_deg = azimuth_raw / 100.0
        base = off + 4
        # 手册表 10：Data Block 1/3/5/7 是通道 1~48，Block 2/4/6/8 是通道
        # 49~96；每两个连续 Block 共享同一个 Azimuth（4 个方位 × 96 通道）。
        ch_offset = (block % 2) * CHANNELS_PER_BLOCK
        for ch_rel in range(CHANNELS_PER_BLOCK):
            channel = ch_offset + ch_rel  # 0-based, 0~95
            d_raw, reflectivity = struct.unpack_from(">HB", data, base + ch_rel * 3)
            if d_raw == 0:
                continue  # 0 表示该通道无有效回波
            dist_m = d_raw * 0.005  # 分辨率 0.5 cm
            if dist_m > MAX_VALID_DISTANCE_M:
                continue
            vertical_deg = vert_lut[channel]

            az = azimuth_deg
            if apply_delay_compensation:
                # 发射延时补偿：晚发射的通道其测得方位需前移（旋转 3600°/s）
                az += _CHANNEL_DELAY_US[channel] * _ROTATION_DEG_PER_US
            if horiz_lut is not None:
                az += horiz_lut[channel]

            # 障碍扇区图：只用接近水平的通道，并把斜距投影到水平面
            if vertical_deg <= max_vertical_deg:
                horizontal_dist = dist_m * math.cos(math.radians(vertical_deg))
                sectors.add(az, horizontal_dist)

            vert_r = math.radians(vertical_deg)
            az_r = math.radians(az)
            # 极坐标 → 笛卡尔（手册 §2.5.1 公式，R=0 / Z=0 即光心在原点）
            xy = dist_m * math.cos(vert_r)
            points.append(
                LidarPoint(
                    azimuth_deg=az,
                    vertical_deg=vertical_deg,
                    distance_m=dist_m,
                    reflectivity=reflectivity,
                    channel=channel + 1,
                    x=xy * math.sin(az_r),
                    y=xy * math.cos(az_r),
                    z=dist_m * math.sin(vert_r),
                )
            )

    return frame


def transform_point_cloud(
    points: list[LidarPoint],
    *,
    pitch_deg: float = 0.0,
    roll_deg: float = 0.0,
    lidar_height_m: float = 0.0,
) -> list[LidarPoint]:
    """Apply lidar-mount extrinsics to a decoded point list (F 迁移).

    保持本项目既定的车体坐标系（x 右 / y 前 / z 上，光心原点）：

      * ``lidar_height_m`` —— 光心离地高度，整体平移 ``z``（正 = 抬高）。
        地形 / 视觉目标定位因此拿到「离地高度」而非「相对光心」。
      * ``pitch_deg``       —— 绕 x 轴俯仰（正 = 抬头）。Airy 立装时用于
        补偿安装倾角；旋转后每个点重新投影回 azimuth / vertical / 距离，
        点云的扇区图与地形分析自动使用新姿态。
      * ``roll_deg``        —— 绕 y 轴横滚（正 = 右侧下沉）。用于动态姿态
        校正（不平路面下车体侧倾时保持 z 轴真正指向「上」）。

    外参均为 0 时原样返回（零开销路径）。
    """
    if not points or (pitch_deg == 0.0 and roll_deg == 0.0
                      and lidar_height_m == 0.0):
        return points

    pitch = math.radians(pitch_deg)
    cos_p, sin_p = math.cos(pitch), math.sin(pitch)
    roll = math.radians(roll_deg)
    cos_r, sin_r = math.cos(roll), math.sin(roll)

    out: list[LidarPoint] = []
    for p in points:
        x, y, z = p.x, p.y, p.z
        if pitch:
            # 绕 +x 轴俯仰：y/z 旋转，x 不变
            y, z = y * cos_p - z * sin_p, y * sin_p + z * cos_p
        if roll:
            # 绕 +y 轴横滚：x/z 旋转，y 不变
            x, z = x * cos_r + z * sin_r, -x * sin_r + z * cos_r
        if lidar_height_m:
            z += lidar_height_m

        hd = math.hypot(x, y)
        dist = math.hypot(hd, z)
        if dist <= 0:
            continue
        az = math.degrees(math.atan2(x, y)) % 360.0
        vert = math.degrees(math.atan2(z, hd))
        out.append(
            LidarPoint(
                azimuth_deg=az,
                vertical_deg=vert,
                distance_m=dist,
                reflectivity=p.reflectivity,
                channel=p.channel,
                x=x,
                y=y,
                z=z,
            )
        )
    return out


class AiryLidar:
    """UDP receiver for RoboSense Airy MSOP packets.

    Usage::

        lidar = AiryLidar(host="0.0.0.0", port=6699, mount_yaw_deg=0.0)
        lidar.start()
        # ... periodically check lidar.latest_frame / lidar.nearest_in_range ...
        lidar.stop()
    """

    NO_DATA_STALE_S: float = 0.4  # 超过该秒数未收到帧视为雷达离线（~10Hz 丢 4 帧）
    source: str = "udp"

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = MSOP_PORT,
        *,
        max_vertical_deg: float = DEFAULT_MAX_VERTICAL_DEG,
        sector_count: int = 360,
        mount_yaw_deg: float = 0.0,
        apply_delay_compensation: bool = True,
        window_frames: int = 8,
        enable_difop: bool = True,
        pitch_deg: float = 0.0,
        lidar_height_m: float = 0.0,
        self_mask: Optional[SelfMaskConfig] = None,
        attitude_source: Optional[AttitudeSource] = None,
    ) -> None:
        self._host = host
        self._port = port
        self._max_vertical_deg = max_vertical_deg
        self._sector_count = sector_count
        # 雷达 0° 到车头方向的偏置：把扇区索引统一到「车头=0°，左转为正」
        self._mount_yaw_deg = mount_yaw_deg % 360.0
        self._apply_delay_compensation = apply_delay_compensation
        # 滑动窗口帧数：累积最近 N 帧点云成 360° 完整障碍图 + 地形剖面，
        # 抗稀疏与噪点（碎石/扬尘）。
        self._window_frames = max(1, window_frames)
        # 安装外参（F：迁移组员 configure_lidar_mount 的参数化能力，保持
        # 默认 0 不变）。lidar_height_m 把点云 z 平移为「离地高度」；
        # pitch_deg 绕 x 轴俯仰旋转（Airy 立装时用于补偿雷达安装倾角）。
        self._pitch_deg = float(pitch_deg) % 360.0
        self._lidar_height_m = float(lidar_height_m)
        self._self_mask = self_mask
        # 动态姿态源（IMU）：不平路面实时校正点云水平基准。默认 None（零开销）。
        self._attitude_source = attitude_source

        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind((host, port))
            self._sock.settimeout(0.5)
        except OSError as exc:
            raise LidarError(
                f"无法绑定雷达 UDP 端口 {port}: {exc}\n"
                "请确认没有其他程序占用该端口，且已为雷达网卡配置静态 IP。"
            ) from exc

        # DIFOP（出厂标定）socket：A-迁移。DIFOP 端口 7788，独立绑定；
        # 收不到时保持手册默认表，绝不阻塞点云。
        self._enable_difop = enable_difop
        self._difop_sock: Optional[socket.socket] = None
        self._difop: Optional[DifopCalibration] = None
        if enable_difop:
            try:
                self._difop_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._difop_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._difop_sock.bind((host, DIFOP_PORT))
                self._difop_sock.settimeout(0.5)
            except OSError:
                logger.warning(
                    "DIFOP 端口 %d 绑定失败（可能被占用），继续用手册默认角度标定",
                    DIFOP_PORT,
                )
                self._difop_sock = None

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._latest_frame: Optional[ScanFrame] = None
        self._last_frame_at: float = 0.0
        self._frame_count: int = 0
        self._packet_count: int = 0
        self._bad_packet_count: int = 0

        # 累积窗口（受 _lock 保护）
        self._window: deque[list[LidarPoint]] = deque(maxlen=self._window_frames)
        self._accum_sectors = AccumulatingSectors(max(72, self._sector_count))
        self._terrain = TerrainProfile()

    # -- lifecycle -------------------------------------------------------

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._rx_loop, name="lidar-rx", daemon=True)
        self._thread.start()
        logger.info("LiDAR receiver started on %s:%d", self._host, self._port)

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.5)
        for sock in (self._sock, self._difop_sock):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass

    # -- accessors -------------------------------------------------------

    @property
    def calibration(self) -> Optional[DifopCalibration]:
        """Latest factory calibration parsed from DIFOP (None = 未收到)."""
        with self._lock:
            return self._difop

    @property
    def using_difop_calibration(self) -> bool:
        """True when DIFOP angle calibration is active (vs manual table)."""
        with self._lock:
            return bool(
                self._difop is not None
                and self._difop.vertical_angles_deg is not None
            )

    @property
    def latest_frame(self) -> Optional[ScanFrame]:
        with self._lock:
            return self._latest_frame

    @property
    def is_receiving(self) -> bool:
        return time.time() - self._last_frame_at < self.NO_DATA_STALE_S

    @property
    def frame_count(self) -> int:
        return self._frame_count

    @property
    def packet_count(self) -> int:
        return self._packet_count

    @property
    def bad_packet_count(self) -> int:
        return self._bad_packet_count

    def nearest_in_range(self, center_deg: float, width_deg: float,
                         quantile: float = 0.25) -> Optional[float]:
        """Nearest obstacle (m) within ``width_deg`` around ``center_deg``.

        Angles are in the vehicle frame (0° = front).  Uses the accumulated
        time-window map with p25 low-quantile robustness, so transient
        gravel/dust spikes do not trigger phantom obstacles.
        """
        with self._lock:
            if not self._window:
                return None
            return self._accum_sectors.nearest_in_range(center_deg, width_deg, quantile)

    def sector_points(self) -> list[tuple[float, float]]:
        """Occupied-sector polar points [(azimuth_deg, distance_m)].

        车体系（0°=车头），来自累积时间窗地图的 p25 低分位距离。
        供 2D 伪地图（``occupancy.OccupancyGrid``）做射线栅格化。
        """
        with self._lock:
            if not self._window:
                return []
            return self._accum_sectors.occupied_polar()

    def point_cloud(
        self,
        max_points: int = 3000,
        max_range_m: float = 60.0,
    ) -> dict:
        """Latest-frame point cloud snapshot for cloud-side visualization.

        返回 ``{online, pointCount, points: [[x, y, z, intensity], ...]}``，
        车体系（x 右 / y 前 / z 上），等间隔子采样到 ``max_points``。
        """
        with self._lock:
            frame = self._latest_frame
        return point_cloud_snapshot(frame, max_points=max_points,
                                    max_range_m=max_range_m)

    # -- terrain (stage-2: rugged terrain traversability) ---------------

    @property
    def terrain(self) -> TerrainProfile:
        """Latest accumulated terrain profile (vehicle frame angles)."""
        with self._lock:
            return self._terrain

    def terrain_sector(self, angle_deg: float,
                       step_limit_m: float = DEFAULT_STEP_LIMIT_M) -> TerrainSectorResult:
        """Traversability verdict for the sector centred on ``angle_deg``.

        0° = 车头方向。区分：台阶/岩壁（不可通行）、坡面（降速）、
        碎石（低矮，通过 max_height_m 判定）、坑/黑洞（unclear）。
        """
        with self._lock:
            return self._terrain.sector(angle_deg, step_limit_m)

    # -- internal --------------------------------------------------------

    def _rx_loop(self) -> None:
        while not self._stop_event.is_set():
            # 轮询 MSOP（点云）+ DIFOP（标定）两个 UDP 口。select 同时等待，
            # 一个口收包不会阻塞另一个。
            socks = [self._sock]
            if self._difop_sock is not None:
                socks.append(self._difop_sock)
            try:
                readable, _, _ = select.select(socks, [], [], 0.5)
            except (OSError, ValueError):
                if self._stop_event.is_set():
                    break
                time.sleep(0.2)
                continue

            for sock in readable:
                try:
                    data, _addr = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                except OSError:
                    if self._stop_event.is_set():
                        break
                    logger.warning("LiDAR socket error, retrying...")
                    time.sleep(0.2)
                    continue

                if sock is self._difop_sock:
                    self._handle_difop(data)
                else:
                    self._handle_msop(data)

    def _handle_difop(self, data: bytes) -> None:
        """Parse a DIFOP packet and apply factory angle calibration (A)."""
        calib = parse_difop_packet(data)
        if calib is None:
            self._bad_packet_count += 1
            return
        if calib.vertical_angles_deg is not None:
            prev = self._difop
            changed = (
                prev is None
                or prev.vertical_angles_deg is None
                or prev.rps != calib.rps
                or prev.install_mode != calib.install_mode
            )
            if changed:
                logger.info(
                    "DIFOP 标定已加载: rps=%.1f install=%d fov=[%.1f, %.1f] deg, "
                    "垂直角标定 %d 通道",
                    calib.rps, calib.install_mode,
                    calib.fov_start_deg, calib.fov_end_deg,
                    len(calib.vertical_angles_deg),
                )
        with self._lock:
            self._difop = calib

    def _handle_msop(self, data: bytes) -> None:
        self._packet_count += 1
        if len(data) != MSOP_PACKET_SIZE:
            self._bad_packet_count += 1
            return

        # 有 DIFOP 标定时用真实通道角度替代手册表
        vert_lut = horiz_lut = None
        with self._lock:
            if self._difop is not None:
                vert_lut = self._difop.vertical_angles_deg
                horiz_lut = self._difop.horizontal_angles_deg
        try:
            frame = parse_msop_packet(
                data,
                max_vertical_deg=self._max_vertical_deg,
                sector_count=self._sector_count,
                apply_delay_compensation=self._apply_delay_compensation,
                vertical_angles_deg=vert_lut,
                horizontal_angles_deg=horiz_lut,
            )
        except ValueError:
            self._bad_packet_count += 1
            return

        # 动态姿态校正（IMU）：不平路面下车体侧倾/俯仰 → 校正点云水平基准，
        # 避免把「车体倾斜导致的坡面」误判成台阶/悬崖。
        dyn_roll = dyn_pitch = 0.0
        if self._attitude_source is not None:
            try:
                att = self._attitude_source()
            except Exception:
                att = None
            if att is not None and len(att) >= 2:
                dyn_roll, dyn_pitch = float(att[0]), float(att[1])

        # 安装外参（F）：俯仰旋转 + 离地高度平移 + 动态横滚，之后重建扇区图
        if self._pitch_deg or self._lidar_height_m or dyn_roll or dyn_pitch:
            frame.points = transform_point_cloud(
                frame.points,
                pitch_deg=self._pitch_deg + dyn_pitch,
                roll_deg=dyn_roll,
                lidar_height_m=self._lidar_height_m,
            )
        # 自扫硬件过滤（C）：剔除安装支架固定点，避免污染扇区图/地形
        if self._self_mask is not None and self._self_mask.enabled:
            frame.points = filter_self_hardware(frame.points, self._self_mask)
        if self._pitch_deg or self._lidar_height_m or dyn_roll or dyn_pitch or (
                self._self_mask is not None and self._self_mask.enabled):
            new_map = ObstacleSectors(self._sector_count)
            for p in frame.points:
                hd = p.distance_m * math.cos(math.radians(p.vertical_deg))
                new_map.add(p.azimuth_deg, hd)
            frame.obstacle_sectors = new_map

        # 雷达安装偏置：把扇区索引统一到车体系
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
            # 累积滑动窗口：最近 N 帧点云 → 完整 360° 障碍图 + 地形剖面
            self._window.append(frame.points)
            self._rebuild_accum_locked()

    def _rebuild_accum_locked(self) -> None:
        """Rebuild the accumulated sectors & terrain from the window.

        Caller must hold ``_lock``.  O(window × points) per frame — a few
        thousand points, sub-millisecond; fine for the 10 Hz RX loop.
        """
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
