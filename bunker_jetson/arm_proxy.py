"""底盘网页到机械臂运动进程的代理。

网页进程（通常是 Python 3.13）不 import SDK。笛卡尔 IK 在
``panthera`` 的 Python 3.10 里跑 ``robot_station`` 的运动进程，SDK 目录
指向 ``~/zx/fafu_arm_sdk-main``。串口锁与站控共用，避免两份进程同时开 USB。
"""

from __future__ import annotations

import json
import os
import socket
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

MOTION_LOCK = Path("/tmp/fafu_station_motion.lock")
MOTION_PORT = 9470
DEFAULT_PYTHON = Path("/home/fafu_robot/miniconda3/envs/panthera/bin/python")
DEFAULT_SDK = Path("/home/fafu_robot/zx/fafu_arm_sdk-main")
DEFAULT_STATION = Path(__file__).resolve().parents[2] / "robot_station"
OCCUPIED_MSG = "机械臂串口已被站控占用"
ARM_OPS = frozenset({
    "cart", "home", "grip", "link", "power", "lease", "mode", "clear", "touch", "arm",
})
_TRUE = frozenset({"1", "true", "yes", "on"})
_FALSE = frozenset({"0", "false", "no", "off"})
_KINDS = frozenset({"sim", "fafu", "mock", "auto"})
# Same USB-serial VIDs as fafu_motor.find_likely_debug_boards (ST/FTDI/CH340/CP210x/PL2303).
_ARM_USB_VIDS = frozenset({"0483", "0403", "1a86", "10c4", "067b"})


def _env_flag(env: dict[str, str], name: str) -> bool | None:
    raw = (env.get(name) or "").strip().lower()
    if raw in _TRUE:
        return True
    if raw in _FALSE:
        return False
    return None


def lock_held(path: Path) -> bool:
    """True when another process holds the flock. A leftover file alone is not held."""
    target = Path(path)
    if not target.is_file():
        return False
    try:
        import fcntl
    except ImportError:
        return False
    try:
        with open(target, "a+", encoding="utf-8") as fh:
            try:
                fcntl.flock(fh.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                return True
            fcntl.flock(fh.fileno(), fcntl.LOCK_UN)
            return False
    except OSError:
        return False


def list_arm_serial_ports() -> list[str]:
    """ttyACM/ttyUSB nodes whose USB VID is an arm debug board. Sorted, stable."""
    found: list[str] = []
    for pattern in ("ttyACM*", "ttyUSB*"):
        for node in sorted(Path("/dev").glob(pattern)):
            vendor = Path(f"/sys/class/tty/{node.name}/device/../idVendor")
            try:
                vid = vendor.read_text(encoding="utf-8").strip().lower()
            except OSError:
                continue
            if vid in _ARM_USB_VIDS:
                found.append(str(node))
    return found


def port_open(port: int, host: str = "127.0.0.1") -> bool:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(0.15)
    try:
        return sock.connect_ex((host, int(port))) == 0
    finally:
        sock.close()


def clamp_cart(msg: dict[str, Any]) -> dict[str, Any]:
    """Keep one keyboard tick inside the SDK step (5 mm / 1.7°) with headroom."""
    out = dict(msg)
    dxyz: list[float] = []
    raw_xyz = list(msg.get("dxyz") or [])
    for i in range(3):
        try:
            val = float(raw_xyz[i]) if i < len(raw_xyz) else 0.0
        except (TypeError, ValueError):
            val = 0.0
        dxyz.append(max(-0.02, min(0.02, val)))
    drpy: list[float] = []
    raw_rpy = list(msg.get("drpy") or [])
    for i in range(3):
        try:
            val = float(raw_rpy[i]) if i < len(raw_rpy) else 0.0
        except (TypeError, ValueError):
            val = 0.0
        drpy.append(max(-5.0, min(5.0, val)))
    out["dxyz"] = dxyz
    out["drpy"] = drpy
    return out


class _MotionLine:
    """JSON-lines client for robot_station motion TCP. Stdlib only."""

    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = int(port)
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._snap: dict[str, Any] = {}
        self._ready: dict[str, Any] = {}
        self._stop = threading.Event()
        self._rx: threading.Thread | None = None
        self._pending: dict[int, tuple[threading.Event, dict | None]] = {}
        self._req = 0
        self.connected = False

    def connect(self, timeout_s: float = 8.0) -> None:
        deadline = time.monotonic() + timeout_s
        last_err: Exception | None = None
        while time.monotonic() < deadline:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1.0)
            try:
                sock.connect((self.host, self.port))
                sock.settimeout(0.5)
                self._sock = sock
                self.connected = True
                self._stop.clear()
                self._rx = threading.Thread(target=self._read_loop, name="arm-rx", daemon=True)
                self._rx.start()
                return
            except OSError as exc:
                last_err = exc
                try:
                    sock.close()
                except OSError:
                    pass
                time.sleep(0.15)
        raise ConnectionError(f"连不上运动进程 {self.host}:{self.port}: {last_err}")

    def close(self) -> None:
        self._stop.set()
        self.connected = False
        sock = self._sock
        self._sock = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        if self._rx is not None:
            self._rx.join(timeout=1.0)
            self._rx = None

    def latest(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._snap)

    def send(self, msg: dict[str, Any]) -> None:
        data = (json.dumps(msg, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")
        with self._send_lock:
            with self._lock:
                sock = self._sock
            if sock is None:
                return
            try:
                sock.sendall(data)
            except OSError:
                self.connected = False

    def call(self, msg: dict[str, Any], timeout_s: float = 2.0) -> dict[str, Any]:
        with self._lock:
            self._req += 1
            req = self._req
        ev = threading.Event()
        with self._lock:
            self._pending[req] = (ev, None)
        payload = dict(msg)
        payload["req"] = req
        self.send(payload)
        if not ev.wait(timeout_s):
            with self._lock:
                self._pending.pop(req, None)
            return {"t": "ack", "ok": False, "error": "运动进程无应答", "op": msg.get("t")}
        with self._lock:
            pair = self._pending.pop(req, None)
        if not pair or pair[1] is None:
            return {"t": "ack", "ok": False, "error": "运动进程无应答", "op": msg.get("t")}
        return pair[1]

    def _read_loop(self) -> None:
        buf = b""
        sock = self._sock
        if sock is None:
            return
        while not self._stop.is_set():
            try:
                chunk = sock.recv(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                if not raw.strip():
                    continue
                try:
                    msg = json.loads(raw.decode("utf-8"))
                except (json.JSONDecodeError, UnicodeDecodeError):
                    continue
                if not isinstance(msg, dict):
                    continue
                kind = msg.get("t")
                if kind == "snap":
                    data = msg.get("d")
                    if isinstance(data, dict):
                        with self._lock:
                            self._snap = data
                elif kind == "ready":
                    with self._lock:
                        self._ready = msg
                elif kind == "ack":
                    req = msg.get("req")
                    with self._lock:
                        pair = self._pending.get(int(req)) if req is not None else None
                        if pair is not None:
                            self._pending[int(req)] = (pair[0], msg)
                            pair[0].set()
        self.connected = False


class ArmProxy:
    """Spawn or refuse the motion process, then forward lease / cart / home."""

    def __init__(
        self,
        *,
        enabled: bool = False,
        kind: str = "sim",
        allow_motion: bool | None = None,
        python: Path | None = None,
        sdk: Path | None = None,
        station: Path | None = None,
        motion_port: int = MOTION_PORT,
        lock_path: Path | None = None,
        log_path: Path | None = None,
        config_path: Path | None = None,
    ) -> None:
        self.enabled = bool(enabled)
        requested = (kind or "sim").strip().lower()
        if requested not in _KINDS:
            requested = "sim"
        self.port = ""
        self.port_note = ""
        if requested == "auto":
            ports = list_arm_serial_ports()
            if len(ports) == 1:
                requested = "fafu"
                self.port = ports[0]
            elif len(ports) > 1:
                requested = "sim"
                self.port_note = "多块调试板，不猜串口：" + "、".join(ports)
            else:
                requested = "sim"
                self.port_note = "没有机械臂 USB，使用仿真"
        self.kind = requested
        if allow_motion is None:
            # auto + one board is the live keyboard path. Plain fafu stays read-only
            # until BUNKER_ARM_ALLOW_MOTION / --arm-allow-motion.
            self.allow_motion = self.kind != "fafu" or bool(self.port)
        else:
            self.allow_motion = bool(allow_motion)
        self.python = Path(python or DEFAULT_PYTHON)
        self.sdk = Path(sdk or DEFAULT_SDK)
        self.station = Path(station or DEFAULT_STATION)
        self.motion_port = int(motion_port)
        self.lock_path = Path(lock_path or MOTION_LOCK)
        self.log_path = Path(log_path or "/tmp/bunker_arm_motion.log")
        self.config_path = Path(config_path) if config_path else None
        self._proc: subprocess.Popen | None = None
        self._spawned = False
        self._line: _MotionLine | None = None
        self._guard = threading.Lock()
        self._started = False
        self.ready = False
        self.occupied = False
        self.error = ""

    @classmethod
    def from_env(cls, env: dict[str, str] | None = None) -> "ArmProxy":
        src = os.environ if env is None else env
        enabled = _env_flag(src, "BUNKER_ARM") is True
        kind = (src.get("BUNKER_ARM_KIND") or "sim").strip().lower()
        allow = _env_flag(src, "BUNKER_ARM_ALLOW_MOTION")
        python = src.get("BUNKER_ARM_PYTHON") or None
        sdk = src.get("FAFU_ARM_SDK") or src.get("BUNKER_ARM_SDK") or None
        station = src.get("BUNKER_ARM_STATION") or None
        return cls(
            enabled=enabled,
            kind=kind,
            allow_motion=allow,
            python=Path(python) if python else None,
            sdk=Path(sdk) if sdk else None,
            station=Path(station) if station else None,
        )

    def status(self) -> dict[str, Any]:
        with self._guard:
            return {
                "enabled": self.enabled,
                "kind": self.kind,
                "ready": self.ready,
                "occupied": self.occupied,
                "allow_motion": self.allow_motion,
                "error": self.error,
                "port": self.motion_port,
                "serial": self.port,
                "note": self.port_note,
            }

    def command(self) -> list[str]:
        """Argv for the motion child. Live USB uses ``--arm fafu --allow-motion``."""
        cmd = [
            str(self.python),
            "-m", "robot_station.service",
            "--motion-only",
            "--no-browser",
            "--arm", self.kind,
        ]
        if self.config_path is not None:
            cmd += ["--config", str(self.config_path)]
        if self.allow_motion:
            cmd.append("--allow-motion")
        return cmd

    def child_env(self) -> dict[str, str]:
        env = os.environ.copy()
        root = str(self.station)
        env["PYTHONPATH"] = root + os.pathsep + env.get("PYTHONPATH", "")
        env["PYTHONUNBUFFERED"] = "1"
        env["FAFU_ARM_SDK"] = str(self.sdk)
        env["STATION_NO_BROWSER"] = "1"
        if self.kind == "fafu":
            env["STATION_ALLOW_LIVE_ARM"] = "1"
        else:
            env.pop("STATION_ALLOW_LIVE_ARM", None)
        if self.port:
            env["STATION_ARM_PORT"] = self.port
        else:
            env.pop("STATION_ARM_PORT", None)
        return env

    def start(self) -> None:
        with self._guard:
            if self._started:
                return
            self._started = True
        if not self.enabled:
            self.error = "机械臂未启用。设置 BUNKER_ARM=1 后重启网页（仿真 BUNKER_ARM_KIND=sim，真机 fafu）。"
            return
        if lock_held(self.lock_path) or port_open(self.motion_port):
            self.occupied = True
            self.error = OCCUPIED_MSG
            return
        if not self.python.is_file():
            self.error = f"找不到 Python 3.10：{self.python}"
            return
        if not self.station.is_dir():
            self.error = f"找不到站控目录：{self.station}"
            return
        if self.kind == "fafu" and not (self.sdk / "fafu_robot_python").is_dir():
            self.error = f"找不到机械臂 SDK：{self.sdk}"
            return
        log_fh = None
        try:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            log_fh = open(self.log_path, "a", encoding="utf-8")
            proc = subprocess.Popen(
                self.command(),
                cwd=str(self.station),
                env=self.child_env(),
                stdout=log_fh,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except OSError as exc:
            if log_fh is not None:
                log_fh.close()
            self.error = f"运动进程启动失败：{exc}"
            return
        self._proc = proc
        self._spawned = True
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                self.error = "运动进程已退出：" + _tail(self.log_path)
                try:
                    proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    pass
                self._proc = None
                self._spawned = False
                return
            if port_open(self.motion_port):
                break
            time.sleep(0.1)
        else:
            self.error = "运动进程未在端口监听：" + _tail(self.log_path)
            self.stop()
            return
        line = _MotionLine("127.0.0.1", self.motion_port)
        try:
            line.connect(timeout_s=8.0)
        except ConnectionError as exc:
            self.error = str(exc)
            self.stop()
            return
        self._line = line
        self.ready = True
        self.error = ""

    def stop(self) -> None:
        line = self._line
        self._line = None
        self.ready = False
        if line is not None:
            line.close()
        proc = self._proc
        self._proc = None
        if proc is not None and self._spawned:
            if proc.poll() is None:
                try:
                    proc.send_signal(2)  # SIGINT — motion process releases the serial lock
                except OSError:
                    pass
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                try:
                    proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    pass
        self._spawned = False

    def latest(self) -> dict[str, Any] | None:
        line = self._line
        if line is None or not line.connected:
            return None
        snap = line.latest()
        return snap or None

    def add_client(self, cid: str) -> None:
        line = self._line
        if line is None or not cid:
            return
        line.send({"t": "add", "cid": cid})

    def drop_client(self, cid: str) -> None:
        line = self._line
        if line is None or not cid:
            return
        line.send({"t": "drop", "cid": cid})

    def estop(self, reason: str = "operator") -> None:
        line = self._line
        if line is None:
            return
        line.send({"t": "e", "reason": reason})

    def call(self, cid: str, msg: dict[str, Any]) -> dict[str, Any]:
        kind = str(msg.get("t") or "")
        if not self.enabled:
            return {"t": "ack", "ok": False, "error": self.error or "机械臂未启用", "op": kind}
        if self.occupied:
            return {"t": "ack", "ok": False, "error": OCCUPIED_MSG, "op": kind}
        line = self._line
        if line is None or not self.ready:
            return {"t": "ack", "ok": False, "error": self.error or "运动进程未就绪", "op": kind}
        payload = dict(msg)
        payload["cid"] = cid
        if kind == "cart":
            payload = clamp_cart(payload)
            payload["cid"] = cid
        if kind in ("home", "grip", "link", "power", "lease", "mode", "clear", "arm", "cart"):
            timeout = 25.0 if kind in ("home", "link", "mode") else 2.0
            return line.call(payload, timeout_s=timeout)
        if kind == "touch":
            line.send(payload)
            return {"t": "ack", "ok": True, "error": None, "op": "touch"}
        return {"t": "ack", "ok": False, "error": f"不支持的机械臂指令 {kind}", "op": kind}


def _tail(path: Path, limit: int = 400) -> str:
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""
    text = text.strip()
    if len(text) <= limit:
        return text
    return text[-limit:]
