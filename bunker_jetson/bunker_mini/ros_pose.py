"""ROS TF ``map``→``base_link`` → :class:`PoseSource`（conda Python 不 import rclpy）。

工控机 conda 环境没有 rclpy。定位位姿由系统 Python 桥
``maps/tf_pose_bridge.sh`` 经 Unix 套接字以 JSON 行转发。
SSH 已开 ``start_mola_localization.sh`` 且 TF 树里有 ``map`` 时，
:meth:`TfPoseSource.get_pose` 才返回地图系 ``(x, y, yaw_deg)``。
"""

from __future__ import annotations

import json
import logging
import math
import os
import signal
import socket
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Optional

logger = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parents[2]
BRIDGE_SH = REPO_ROOT / "maps" / "tf_pose_bridge.sh"
STALE_S = 0.80
PACKET_STALE_S = 2.0


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """ROS 四元数 → yaw（绕 Z，弧度），与 tf2 的 getRPY 一致。"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class TfPoseSource:
    """Unix 套接字上的 TF 位姿源。桥进程未开或 lookup 失败时 ``get_pose`` 为 None。"""

    def __init__(
        self,
        sock_path: Optional[str] = None,
        *,
        spawn_bridge: bool = True,
        bridge_argv: Optional[list[str]] = None,
        stale_s: float = STALE_S,
        parent_frame: str = "map",
        child_frame: str = "base_link",
    ) -> None:
        self._sock_path = sock_path or f"/tmp/bunker_tf_pose.{os.getpid()}.sock"
        self._spawn_bridge = bool(spawn_bridge)
        self._bridge_argv = bridge_argv
        self._stale_s = float(stale_s)
        self._parent = parent_frame
        self._child = child_frame
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._server: Optional[socket.socket] = None
        self._client: Optional[socket.socket] = None
        self._proc: Optional[subprocess.Popen] = None
        self._last: dict[str, Any] = {}
        self._last_at: float = 0.0
        self._started = False

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        try:
            os.unlink(self._sock_path)
        except FileNotFoundError:
            pass
        except OSError as exc:
            logger.warning("无法清理 TF 位姿套接字 %s: %s", self._sock_path, exc)
            return
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            srv.bind(self._sock_path)
            srv.listen(1)
            srv.settimeout(0.5)
        except OSError as exc:
            srv.close()
            logger.warning("无法监听 TF 位姿套接字 %s: %s", self._sock_path, exc)
            return
        self._server = srv
        if self._spawn_bridge:
            try:
                self._proc = self._spawn()
            except Exception:
                logger.warning("TF 位姿桥未启动（SSH 定位未开时这是正常的）", exc_info=True)
                self._proc = None
        self._thread = threading.Thread(
            target=self._rx_loop, name="tf-pose", daemon=True,
        )
        self._thread.start()
        self._started = True

    def _spawn(self) -> subprocess.Popen:
        argv = self._bridge_argv
        if argv is None:
            if not BRIDGE_SH.is_file():
                raise FileNotFoundError(f"找不到 TF 位姿桥: {BRIDGE_SH}")
            argv = [
                "bash", str(BRIDGE_SH),
                "--socket", self._sock_path,
                "--parent", self._parent,
                "--child", self._child,
            ]
        proc = subprocess.Popen(
            argv,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        time.sleep(0.15)
        code = proc.poll()
        if code is not None:
            raise RuntimeError(f"tf_pose_bridge 退出: exit {code}")
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
        self._started = False

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            srv = self._server
            if srv is None:
                break
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    break
                time.sleep(0.2)
                continue
            conn.settimeout(0.5)
            self._client = conn
            buf = b""
            try:
                while not self._stop.is_set():
                    try:
                        chunk = conn.recv(4096)
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self._on_line(line)
            finally:
                try:
                    conn.close()
                except OSError:
                    pass
                self._client = None

    def _on_line(self, raw: bytes) -> None:
        text = raw.decode("utf-8", "replace").strip()
        if not text:
            return
        try:
            msg = json.loads(text)
        except json.JSONDecodeError:
            return
        if not isinstance(msg, dict):
            return
        with self._lock:
            self._last = msg
            self._last_at = time.monotonic()

    def get_pose(self) -> Optional[tuple[float, float, float]]:
        with self._lock:
            msg = dict(self._last)
            age = time.monotonic() - self._last_at if self._last_at else 1e9
        if age > self._stale_s:
            return None
        if not msg.get("ok"):
            return None
        stamp_age = msg.get("stampAgeS")
        if stamp_age is not None:
            try:
                if abs(float(stamp_age)) > self._stale_s:
                    return None
            except (TypeError, ValueError):
                return None
        try:
            x = float(msg["x"])
            y = float(msg["y"])
            yaw = float(msg.get("yawDeg") if msg.get("yawDeg") is not None else msg["yaw_deg"])
        except (KeyError, TypeError, ValueError):
            return None
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)):
            return None
        return x, y, yaw

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            msg = dict(self._last)
            age = time.monotonic() - self._last_at if self._last_at else None
        pose = self.get_pose()
        packet_fresh = age is not None and age < PACKET_STALE_S
        stale_tf = False
        stamp_age = msg.get("stampAgeS")
        if stamp_age is not None:
            try:
                stale_tf = abs(float(stamp_age)) > self._stale_s
            except (TypeError, ValueError):
                stale_tf = True
        # locOnline 只信 map 帧 + 未过期 TF，不信桥进程心跳（online）。
        has_map = packet_fresh and bool(msg.get("hasMap")) and not stale_tf
        online = bool(self._started) and packet_fresh
        out: dict[str, Any] = {
            "bridge": bool(self._started),
            "online": online,
            "hasMap": has_map and packet_fresh,
            "locked": pose is not None,
            "ageS": round(age, 3) if age is not None else None,
            "parent": msg.get("parent") or self._parent,
            "child": msg.get("child") or self._child,
            "reason": msg.get("reason") or "",
        }
        if pose is not None:
            out["x"], out["y"], out["yawDeg"] = (
                round(pose[0], 3), round(pose[1], 3), round(pose[2], 2),
            )
        elif msg.get("ok") and msg.get("x") is not None:
            try:
                out["x"] = round(float(msg["x"]), 3)
                out["y"] = round(float(msg["y"]), 3)
                out["yawDeg"] = round(float(msg.get("yawDeg") or 0.0), 2)
            except (TypeError, ValueError):
                pass
        return out
