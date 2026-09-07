"""Subscribe to ``/rslidar_points`` without binding Airy MSOP UDP 6699.

rslidar_sdk owns unicast UDP 6699. This module is the agent-side half:

  * binary frames on a Unix stream socket (no rclpy in conda Python)
  * sensor-frame XYZ → agent vehicle frame (x 右 / y 前 / z 上)
  * AiryLidar-compatible accessors for obstacle / occupancy / goto

The ROS Humble process is ``maps/rslidar_cloud_bridge.sh`` (system Python).
"""

from __future__ import annotations

import logging
import math
import os
import signal
import socket
import struct
import subprocess
import threading
import time
from collections import deque
from pathlib import Path
from typing import Iterable, Optional

from .lidar import (
    OA_ACCUM_MAX_VERTICAL_DEG,
    AccumulatingSectors,
    LidarError,
    LidarPoint,
    ObstacleSectors,
    ScanFrame,
    SelfMaskConfig,
    filter_self_hardware,
    point_cloud_snapshot,
    transform_point_cloud,
)
from .terrain import DEFAULT_STEP_LIMIT_M, TerrainProfile, TerrainSectorResult

logger = logging.getLogger(__name__)

MAGIC = b"RSC1"
HEADER_STRUCT = struct.Struct("<4sI")
POINT_STRUCT = struct.Struct("<ffff")  # sensor x y z intensity
MAX_POINTS_PER_FRAME = 20_000
DEFAULT_HEIGHT_M = 0.365  # 建图记录：光心离地，地面约在传感器 y≈-0.36 m
REPO_ROOT = Path(__file__).resolve().parents[2]
BRIDGE_SH = REPO_ROOT / "maps" / "rslidar_cloud_bridge.sh"


def encode_cloud_frame(points: Iterable[tuple[float, float, float, float]]) -> bytes:
    """Pack sensor-frame points ``(x, y, z, intensity)`` into one socket frame."""
    payload = bytearray()
    n = 0
    for x, y, z, intensity in points:
        if n >= MAX_POINTS_PER_FRAME:
            break
        payload += POINT_STRUCT.pack(float(x), float(y), float(z), float(intensity))
        n += 1
    return HEADER_STRUCT.pack(MAGIC, len(payload)) + bytes(payload)


def decode_cloud_payload(payload: bytes) -> list[tuple[float, float, float, float]]:
    """Unpack a frame payload into sensor-frame tuples. Truncates a short tail."""
    step = POINT_STRUCT.size
    n = len(payload) // step
    out: list[tuple[float, float, float, float]] = []
    for i in range(n):
        x, y, z, intensity = POINT_STRUCT.unpack_from(payload, i * step)
        out.append((x, y, z, intensity))
    return out


def rslidar_to_vehicle(
    x_s: float,
    y_s: float,
    z_s: float,
    *,
    height_m: float = DEFAULT_HEIGHT_M,
    extra_yaw_deg: float = 0.0,
) -> tuple[float, float, float]:
    """``rslidar`` sensor XYZ → agent vehicle (x 右 / y 前 / z 上).

    建图记录确认的几何（不要用未目视确认的 yaw 45° 当默认）：

      * 传感器 ``+Z`` 光轴朝前，``+Y`` 朝上，``+X`` 朝左（ROS yaw+90 后左右不颠倒）
      * 等价于 ROS ``base_link``：``(z_s, x_s, y_s + height)`` 再转到本项目车体系

    ``extra_yaw_deg`` 绕车体竖直轴、左转为正，对应 ``BUNKER_LIDAR_ROS_YAW_DEG``。
    """
    x = -x_s
    y = z_s
    z = y_s + height_m
    if extra_yaw_deg:
        psi = math.radians(extra_yaw_deg)
        c, s = math.cos(psi), math.sin(psi)
        x, y = x * c - y * s, x * s + y * c
    return (x, y, z)


def vehicle_points_from_sensor(
    sensor_pts: Iterable[tuple[float, float, float, float]],
    *,
    height_m: float = DEFAULT_HEIGHT_M,
    extra_yaw_deg: float = 0.0,
    min_range_m: float = 0.05,
    max_range_m: float = 60.0,
) -> list[LidarPoint]:
    """Build vehicle-frame :class:`LidarPoint` list from sensor XYZ + intensity."""
    out: list[LidarPoint] = []
    for x_s, y_s, z_s, intensity in sensor_pts:
        if not (math.isfinite(x_s) and math.isfinite(y_s) and math.isfinite(z_s)):
            continue
        x, y, z = rslidar_to_vehicle(
            x_s, y_s, z_s, height_m=height_m, extra_yaw_deg=extra_yaw_deg,
        )
        hd = math.hypot(x, y)
        dist = math.hypot(hd, z)
        if dist < min_range_m or dist > max_range_m:
            continue
        az = math.degrees(math.atan2(x, y)) % 360.0
        vert = math.degrees(math.atan2(z, hd)) if hd > 1e-9 else (90.0 if z >= 0 else -90.0)
        refl = int(max(0, min(255, round(intensity))))
        out.append(LidarPoint(
            azimuth_deg=az,
            vertical_deg=vert,
            distance_m=dist,
            reflectivity=refl,
            channel=0,
            x=x, y=y, z=z,
        ))
    return out


def _recv_exact(sock: socket.socket, n: int, *, idle_ok: bool = False) -> bytes | None:
    buf = bytearray()
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except socket.timeout:
            if not buf:
                if idle_ok:
                    raise
                continue
            return None
        except OSError:
            return None
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


class RslidarCloudSource:
    """AiryLidar-compatible source fed by the ROS bridge over a Unix socket."""

    NO_DATA_STALE_S: float = 0.8
    source: str = "ros"

    def __init__(
        self,
        *,
        sock_path: str | None = None,
        spawn_bridge: bool = True,
        bridge_argv: Optional[list[str]] = None,
        topic: str = "/rslidar_points",
        max_points: int = 4000,
        height_m: float = DEFAULT_HEIGHT_M,
        extra_yaw_deg: float = 0.0,
        mount_yaw_deg: float = 0.0,
        pitch_deg: float = 0.0,
        lidar_height_m: float = 0.0,
        self_mask: Optional[SelfMaskConfig] = None,
        sector_count: int = 360,
        window_frames: int = 8,
    ) -> None:
        self._sock_path = sock_path or f"/tmp/bunker_rslidar_cloud.{os.getpid()}.sock"
        self._spawn_bridge = bool(spawn_bridge)
        self._bridge_argv = bridge_argv
        self._topic = topic
        self._max_points = max(1, int(max_points))
        self._height_m = float(height_m)
        self._extra_yaw_deg = float(extra_yaw_deg)
        self._mount_yaw_deg = float(mount_yaw_deg) % 360.0
        self._pitch_deg = float(pitch_deg)
        # ROS 路径已把光心高度并进 rslidar_to_vehicle；此处只叠加 agent 外参俯仰。
        self._lidar_height_m = float(lidar_height_m)
        self._self_mask = self_mask
        self._sector_count = int(sector_count)
        self._window_frames = max(1, int(window_frames))

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._server: Optional[socket.socket] = None
        self._client: Optional[socket.socket] = None
        self._proc: Optional[subprocess.Popen] = None
        self._latest_frame: Optional[ScanFrame] = None
        self._last_frame_at: float = 0.0
        self._frame_count: int = 0
        self._packet_count: int = 0
        self._bad_packet_count: int = 0
        self._window: deque[list[LidarPoint]] = deque(maxlen=self._window_frames)
        self._accum_sectors = AccumulatingSectors(max(72, self._sector_count // 3))
        self._terrain = TerrainProfile()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        try:
            os.unlink(self._sock_path)
        except FileNotFoundError:
            pass
        except OSError as exc:
            raise LidarError(f"无法清理雷达点云套接字 {self._sock_path}: {exc}") from exc
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            srv.bind(self._sock_path)
            srv.listen(1)
            srv.settimeout(0.5)
        except OSError as exc:
            srv.close()
            raise LidarError(f"无法监听雷达点云套接字 {self._sock_path}: {exc}") from exc
        self._server = srv
        if self._spawn_bridge:
            self._proc = self._spawn()
        self._thread = threading.Thread(
            target=self._rx_loop, name="rslidar-cloud", daemon=True,
        )
        self._thread.start()

    def _spawn(self) -> subprocess.Popen:
        argv = self._bridge_argv
        if argv is None:
            if not BRIDGE_SH.is_file():
                raise LidarError(
                    f"MSOP 端口已被占用，需要订 /rslidar_points，但找不到桥脚本: {BRIDGE_SH}"
                )
            argv = [
                "bash", str(BRIDGE_SH),
                "--socket", self._sock_path,
                "--topic", self._topic,
                "--max-points", str(self._max_points),
            ]
        try:
            proc = subprocess.Popen(
                argv,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                start_new_session=True,
            )
        except OSError as exc:
            raise LidarError(f"无法启动 /rslidar_points 桥进程: {exc}") from exc
        time.sleep(0.25)
        code = proc.poll()
        if code is not None:
            err = b""
            try:
                err = proc.stderr.read() if proc.stderr else b""
            except OSError:
                pass
            text = " ".join(err.decode("utf-8", "replace").split())[:240]
            raise LidarError(
                f"MSOP 已被占用，改订 /rslidar_points 失败: exit {code}"
                + (f"（{text}）" if text else "")
            )
        if proc.stderr is not None:
            proc.stderr.close()
        return proc

    def stop(self) -> None:
        self._stop.set()
        proc = self._proc
        self._proc = None
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    proc.kill()
        for sock in (self._client, self._server):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self._client = None
        self._server = None
        if self._thread:
            self._thread.join(timeout=1.5)
            self._thread = None
        try:
            os.unlink(self._sock_path)
        except OSError:
            pass

    @property
    def calibration(self):
        return None

    @property
    def using_difop_calibration(self) -> bool:
        return False

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
        with self._lock:
            if not self._window:
                return None
            return self._accum_sectors.nearest_in_range(center_deg, width_deg, quantile)

    def sector_points(self) -> list[tuple[float, float]]:
        with self._lock:
            if not self._window:
                return []
            return self._accum_sectors.occupied_polar()

    def point_cloud(self, max_points: int = 3000, max_range_m: float = 60.0) -> dict:
        with self._lock:
            frame = self._latest_frame
        return point_cloud_snapshot(frame, max_points=max_points, max_range_m=max_range_m)

    @property
    def terrain(self) -> TerrainProfile:
        with self._lock:
            return self._terrain

    def terrain_sector(self, angle_deg: float,
                       step_limit_m: float = DEFAULT_STEP_LIMIT_M) -> TerrainSectorResult:
        with self._lock:
            return self._terrain.sector(angle_deg, step_limit_m)

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            conn = self._client
            if conn is None:
                srv = self._server
                if srv is None:
                    return
                try:
                    conn, _addr = srv.accept()
                except socket.timeout:
                    self._check_bridge()
                    continue
                except OSError:
                    if self._stop.is_set():
                        return
                    time.sleep(0.2)
                    continue
                conn.settimeout(0.5)
                self._client = conn
            if not self._read_one(conn):
                try:
                    conn.close()
                except OSError:
                    pass
                self._client = None

    def _check_bridge(self) -> None:
        proc = self._proc
        if proc is None or proc.poll() is None:
            return
        err = b""
        try:
            err = proc.stderr.read() if proc.stderr else b""
        except OSError:
            pass
        logger.warning(
            "/rslidar_points 桥进程已退出 %s: %s",
            proc.returncode,
            err.decode("utf-8", "replace").strip()[:300],
        )
        self._proc = None

    def _read_one(self, conn: socket.socket) -> bool:
        try:
            hdr = _recv_exact(conn, HEADER_STRUCT.size, idle_ok=True)
        except socket.timeout:
            return True
        if hdr is None:
            return False
        magic, nbytes = HEADER_STRUCT.unpack(hdr)
        if magic != MAGIC or nbytes > MAX_POINTS_PER_FRAME * POINT_STRUCT.size:
            self._bad_packet_count += 1
            return False
        if nbytes == 0:
            return True
        payload = _recv_exact(conn, nbytes)
        if payload is None:
            return False
        self._ingest(decode_cloud_payload(payload))
        return True

    def _ingest(self, sensor_pts: list[tuple[float, float, float, float]]) -> None:
        self._packet_count += 1
        points = vehicle_points_from_sensor(
            sensor_pts,
            height_m=self._height_m,
            extra_yaw_deg=self._extra_yaw_deg,
        )
        if self._pitch_deg or self._lidar_height_m:
            points = transform_point_cloud(
                points,
                pitch_deg=self._pitch_deg,
                lidar_height_m=self._lidar_height_m,
            )
        if self._self_mask is not None and self._self_mask.enabled:
            points = filter_self_hardware(points, self._self_mask)
        sectors = ObstacleSectors(self._sector_count)
        for p in points:
            hd = p.distance_m * math.cos(math.radians(p.vertical_deg))
            if hd >= 0.05:
                sectors.add((p.azimuth_deg + self._mount_yaw_deg) % 360.0, hd)
        frame = ScanFrame(points=points, obstacle_sectors=sectors, received_at=time.time())
        with self._lock:
            self._latest_frame = frame
            self._last_frame_at = time.time()
            self._frame_count += 1
            self._window.append(points)
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
