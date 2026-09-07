#!/usr/bin/env python3
"""工控机网页控制台：笔记本浏览器 HID 100Hz stick + 任务指令 → :9100。

这不是旧云端那条 JSON ``move`` / WebSocket 任务通道。
浏览器有 keydown/keyup，所以可以真双轴、松键即停；任务走同一条 TCP。

    工控机:  python3 run_local.py          # 会顺带拉起本服务 :9101
    或单独:  python3 teleop_web.py         # 前提是 :9100 已在听

    笔记本:  浏览器打开 http://<工控机局域网IP>:9101
             先输入控制台密码，再点页面按 WASD。任务按钮可下发 fo / goto / 轨迹等。
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import hmac
import json
import os
import secrets
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path
from urllib.parse import parse_qs, unquote

ROOT = Path(__file__).resolve().parent
JETSON = ROOT / "bunker_jetson"
if str(JETSON) not in sys.path:
    sys.path.insert(0, str(JETSON))

from bunker_mini.teleop_tcp import DEFAULT_PORT, DEFAULT_TOKEN, TeleopTcpClient, json_bytes  # noqa: E402
from bunker_mini.tracker import Track, track_web_summary  # noqa: E402
from bunker_mini.qualified_maps import qualified_ids  # noqa: E402

DEFAULT_WEB_PORT = 9101
DEFAULT_WEB_PASSWORD = "fafu123456"
_SID_COOKIE = "bunker_sid"
_SESSION_TTL_S = 12 * 3600
_LOGIN_WINDOW_S = 60.0
_LOGIN_MAX_FAILS = 5
_WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
_MAX_WS = 65535
_WS_VISUAL_MAX = 24_000
_PAGE_PATH = ROOT / "teleop_web.html"
TRACKS = JETSON / "tracks"


def arrived_starts_reverse(payload: dict) -> bool:
    """往返 fb：只有去程完整走完才自动倒放，残缺/倒放本身不得再触发一趟。"""
    data = payload.get("data") if isinstance(payload, dict) else None
    if isinstance(data, dict):
        direction = str(data.get("direction") or "")
        if direction == "reverse" or data.get("phase") in ("dock", "reverse"):
            return False
        if data.get("complete") is False:
            return False
        if direction == "forward":
            return bool(data.get("complete", True)) and not data.get("timeFallback")
    text = str((payload or {}).get("msg") or "")
    if "Reverse" in text or "back at start" in text or "原路返回" in text or "docked" in text:
        return False
    if "interrupted" in text or "approximate" in text or "未完整" in text:
        return False
    return "Track replay completed" in text or "回放完成" in text
_ALLOWED_ACTIONS = frozenset({
    "move", "estop",     "goto", "find_object", "go_home", "cancel", "odom_reset",
    "grasp_done", "map_return", "pose_align", "map_upload",
    "set_wheelbase", "calibrate_wheelbase",
    "lidar_on", "lidar_off", "lidar_status", "lidar_map", "point_cloud",
    "query", "track_record", "track_follow", "track_follow_back", "track_delete",
    "task_submit",
    "autonav_goto", "autonav_select_map", "autonav_map", "autonav_cancel",
})


def _page_bytes() -> bytes:
    return _PAGE_PATH.read_bytes()


def _qualified_stems() -> set[str]:
    try:
        return {x.lower() for x in qualified_ids()}
    except Exception:
        return set()


def _lan_ips() -> list[str]:
    ips: list[str] = []
    try:
        for tok in os.popen("hostname -I 2>/dev/null").read().split():
            if tok.count(".") == 3 and not tok.startswith("127."):
                ips.append(tok)
    except Exception:
        pass

    def _skip_for_laptop(ip: str) -> bool:
        return ip.startswith("192.168.1.") or ip.startswith("172.17.")

    client = [ip for ip in ips if not _skip_for_laptop(ip)]
    other = [ip for ip in ips if _skip_for_laptop(ip)]
    return client + other


def _ws_accept(key: str) -> str:
    raw = hashlib.sha1((key + _WS_GUID).encode("ascii")).digest()
    return base64.b64encode(raw).decode("ascii")


def _cap_xy(items: object, cap: int) -> list:
    if not isinstance(items, list) or cap <= 0:
        return []
    if len(items) <= cap:
        return items
    step = max(1, int((len(items) + cap - 1) / cap))
    return items[::step][:cap]


def _shrink_ws_visual(ev: str, data: dict) -> dict:
    out = dict(data)
    out.pop("text", None)
    if ev == "point_cloud":
        pts = _cap_xy(out.get("points"), 250)
        out["points"] = pts
        out["shown"] = len(pts)
    elif ev == "lidar_map":
        for key in ("occupied", "blocked", "free"):
            arr = out.get(key)
            if isinstance(arr, list):
                out[key] = _cap_xy(arr, 250)
    elif ev == "autonav_map":
        for key in (
            "occupied", "blocked", "free",
            "sketchOccupied", "odomOccupied", "odomBlocked", "odomFree",
            "cloudXY",
        ):
            arr = out.get(key)
            if isinstance(arr, list):
                out[key] = _cap_xy(arr, 400)
    return out


class _WsIdle(Exception):
    """等待下一帧时超时：连接还在，不要当断线急停。"""


def _ws_recv(sock: socket.socket) -> tuple[int, bytes] | None:
    hdr = _recv_exact(sock, 2, idle_ok=True)
    if hdr is None:
        return None
    b0, b1 = hdr[0], hdr[1]
    opcode = b0 & 0x0F
    masked = bool(b1 & 0x80)
    length = b1 & 0x7F
    if length == 126:
        ext = _recv_exact(sock, 2)
        if ext is None:
            return None
        length = struct.unpack(">H", ext)[0]
    elif length == 127:
        ext = _recv_exact(sock, 8)
        if ext is None:
            return None
        length = struct.unpack(">Q", ext)[0]
    if length > _MAX_WS:
        return None
    mask = b""
    if masked:
        mask = _recv_exact(sock, 4)
        if mask is None:
            return None
    data = _recv_exact(sock, length) if length else b""
    if data is None:
        return None
    if masked:
        data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
    return opcode, data


def _recv_exact(sock: socket.socket, n: int, *, idle_ok: bool = False) -> bytes | None:
    buf = bytearray()
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except socket.timeout:
            if idle_ok and not buf:
                raise _WsIdle()
            continue
        except OSError:
            return None
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def _ws_send(sock: socket.socket, opcode: int, payload: bytes) -> None:
    header = bytes([0x80 | (opcode & 0x0F)])
    n = len(payload)
    if n < 126:
        header += bytes([n])
    elif n < 65536:
        header += bytes([126]) + struct.pack(">H", n)
    else:
        header += bytes([127]) + struct.pack(">Q", n)
    sock.sendall(header + payload)


class TeleopWebServer:
    """HTTP 出控制台，WS 收 HID stick + 任务指令，转发到车侧 TCP :9100。"""

    def __init__(
        self,
        tcp_host: str = "127.0.0.1",
        tcp_port: int = DEFAULT_PORT,
        token: str = DEFAULT_TOKEN,
        http_host: str = "0.0.0.0",
        http_port: int = DEFAULT_WEB_PORT,
        v: float = 0.10,
        w: float = 0.40,
        password: str | None = None,
    ) -> None:
        self.tcp_host = tcp_host
        self.tcp_port = int(tcp_port)
        self.token = token
        self.http_host = http_host
        self.http_port = int(http_port)
        self.v_scale = float(v)
        self.w_scale = float(w)
        self._stop = threading.Event()
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._evt_th: threading.Thread | None = None
        self._tcp: TeleopTcpClient | None = None
        self._tcp_lock = threading.Lock()
        self._out_lock = threading.Lock()
        self._ws_lock = threading.Lock()
        self._ws_clients: list[socket.socket] = []
        self._viewers: list[subprocess.Popen] = []
        self._roundtrip_track = ""
        self._last_state: dict | None = None
        self._evt_lock = threading.Lock()
        self._clients = 0
        self.last_rtt_ms = 0.0
        self._warn_cache = ""
        self._warn_t = 0.0
        self._last_ack_vw = None
        self._last_ack_t = 0.0
        raw_pw = password if password is not None else os.environ.get(
            "BUNKER_WEB_PASSWORD", DEFAULT_WEB_PASSWORD
        )
        self._password_digest = hashlib.sha256(raw_pw.encode("utf-8")).digest()
        self._sessions: dict[str, float] = {}
        self._sess_lock = threading.Lock()
        self._fail_at: dict[str, list[float]] = {}

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._ensure_tcp()
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        srv.bind((self.http_host, self.http_port))
        srv.listen(4)
        srv.settimeout(0.5)
        self._sock = srv
        self._stop.clear()
        self._thread = threading.Thread(target=self._accept, name="teleop-web", daemon=True)
        self._thread.start()
        self._evt_th = threading.Thread(target=self._event_loop, name="teleop-web-evt", daemon=True)
        self._evt_th.start()

    def stop(self) -> None:
        self._stop.set()
        self._close_viewers()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._evt_th:
            self._evt_th.join(timeout=1.0)
        with self._ws_lock:
            for sock in list(self._ws_clients):
                try:
                    sock.close()
                except OSError:
                    pass
            self._ws_clients.clear()
        with self._tcp_lock:
            if self._tcp is not None:
                try:
                    self._tcp.stick(0.0, 0.0)
                except Exception:
                    pass
                try:
                    self._tcp.close()
                except Exception:
                    pass
                self._tcp = None

    def urls(self) -> list[str]:
        ips = _lan_ips() or ["<工控机局域网IP>"]
        return [f"http://{ip}:{self.http_port}" for ip in ips]

    def _ws_out(self, sock: socket.socket, payload: dict) -> None:
        raw = json_bytes(payload)
        if raw is None:
            return
        try:
            with self._out_lock:
                _ws_send(sock, 0x1, raw)
        except OSError:
            pass

    def _broadcast(self, payload: dict) -> None:
        with self._ws_lock:
            clients = list(self._ws_clients)
        dead: list[socket.socket] = []
        for sock in clients:
            try:
                self._ws_out(sock, payload)
            except OSError:
                dead.append(sock)
        if dead:
            with self._ws_lock:
                for sock in dead:
                    if sock in self._ws_clients:
                        self._ws_clients.remove(sock)

    def _event_loop(self) -> None:
        while not self._stop.is_set():
            try:
                tcp = self._ensure_tcp()
                with self._evt_lock:
                    events = tcp.poll_events()
                for msg in events:
                    try:
                        self._on_tcp_msg(msg)
                    except Exception:
                        continue
            except OSError:
                with self._tcp_lock:
                    self._tcp = None
                time.sleep(0.2)
                continue
            except Exception:
                time.sleep(0.2)
                continue
            time.sleep(0.05)

    def _state_from_file(self) -> dict | None:
        try:
            with open("/tmp/bunker_chassis_status", "r", encoding="utf-8") as fp:
                info = json.load(fp)
        except Exception:
            return None
        if not isinstance(info, dict):
            return None
        if isinstance(info.get("payload"), dict):
            return info["payload"]
        out = dict(info)
        if "can" not in out:
            out["can"] = {
                "channel": info.get("channel"),
                "interface": "socketcan",
                "link": "up" if info.get("heard") else "?",
                "feedback": bool(info.get("heard")),
            }
        return out

    def _pull_state(self) -> dict | None:
        """主动 query 并在本线程取出 state，不依赖后台事件循环。"""
        file_state = self._state_from_file()
        if file_state:
            self._last_state = {**(self._last_state or {}), **file_state}
        try:
            tcp = self._ensure_tcp()
            tcp.cmd({"action": "query", "ts": int(time.time() * 1000)})
            deadline = time.monotonic() + 0.8
            while time.monotonic() < deadline:
                with self._evt_lock:
                    events = tcp.poll_events()
                for msg in events:
                    try:
                        self._on_tcp_msg(msg)
                    except Exception:
                        continue
                    if msg.get("type") == "state" and isinstance(msg.get("payload"), dict):
                        return msg["payload"]
                time.sleep(0.05)
        except Exception:
            pass
        return self._last_state

    def _on_tcp_msg(self, msg: dict) -> None:
        kind = msg.get("type")
        payload = msg.get("payload") if isinstance(msg.get("payload"), dict) else {}
        if kind == "state":
            self._last_state = payload
            self._broadcast({"t": "state", "payload": payload})
            return
        if kind != "event":
            return
        ev = str(payload.get("event") or "")
        text = str(payload.get("msg") or "")
        if ev in ("point_cloud", "lidar_map", "lidar_status", "autonav_map"):
            data = payload.get("data") if isinstance(payload.get("data"), dict) else None
            if data is None and text.startswith("{"):
                try:
                    parsed = json.loads(text)
                except json.JSONDecodeError:
                    parsed = None
                if isinstance(parsed, dict):
                    data = parsed
            if data is not None:
                data = _shrink_ws_visual(ev, data)
                blob = {"t": ev, "payload": data}
                raw = json_bytes(blob)
                if raw is not None and len(raw) > _WS_VISUAL_MAX and ev in ("lidar_map", "autonav_map"):
                    data = dict(data)
                    data.pop("free", None)
                    blob = {"t": ev, "payload": data}
                    raw = json_bytes(blob)
                if raw is not None and len(raw) <= _WS_VISUAL_MAX:
                    self._broadcast(blob)
            if ev == "lidar_status":
                src = data.get("source") if data else ""
                online = data.get("online") if data else False
                diag = (data or {}).get("diagnosis") or ""
                short = (
                    ("在线" if online else "离线")
                    + (f" · {src}" if src else "")
                    + (f" · {diag}" if diag else "")
                )
                self._broadcast({"t": "event", "payload": {"event": "lidar_status", "msg": short}})
            return
        if ev == "chassis":
            data = payload.get("data")
            if isinstance(data, dict):
                merged = dict(self._last_state or {})
                merged["chassis"] = data
                sys = data.get("system") if isinstance(data.get("system"), dict) else {}
                bms = data.get("bms") if isinstance(data.get("bms"), dict) else {}
                if sys.get("controlMode"):
                    merged["mode"] = sys["controlMode"]
                if sys.get("vehicleState"):
                    merged["vehicleState"] = sys["vehicleState"]
                if bms.get("socPercent") is not None:
                    merged["battery"] = int(bms["socPercent"]) / 100.0
                self._last_state = merged
                self._broadcast({"t": "state", "payload": merged})
            return
        self._broadcast({"t": "event", "payload": payload})
        if ev == "arrived" and self._roundtrip_track:
            if arrived_starts_reverse(payload):
                name = self._roundtrip_track
                self._roundtrip_track = ""
                self._dispatch_cmd({
                    "action": "track_follow",
                    "trackId": name,
                    "reverse": True,
                    "bypassGuard": True,
                })
                self._broadcast({
                    "t": "event",
                    "payload": {"event": "track_follow", "msg": "往返：已到终点，正在原路返回"},
                })
            elif "Reverse" in str(payload.get("msg") or "") or (
                isinstance(payload.get("data"), dict)
                and payload["data"].get("direction") == "reverse"
            ):
                self._roundtrip_track = ""

    def _list_tracks(self) -> list[dict]:
        items: list[dict] = []
        seen: set[str] = set()
        dirs = []
        for folder in (JETSON / "tracks", ROOT / "tracks"):
            try:
                key = str(folder.resolve())
            except OSError:
                key = str(folder)
            if key in seen:
                continue
            seen.add(key)
            dirs.append(folder)
        by_name: dict[str, dict] = {}
        for folder in dirs:
            try:
                names = sorted(f for f in os.listdir(folder) if f.endswith(".json"))
            except FileNotFoundError:
                continue
            for n in names:
                path = folder / n
                rec = {
                    "name": n[:-5],
                    "duration": 0.0,
                    "waypoints": 0,
                    "odometerSource": "unknown",
                    "driveMode": "unknown",
                    "hasOdo": False,
                    "returnOk": False,
                    "dockOk": False,
                    "reasons": ["unreadable"],
                    "startPose": None,
                    "distanceM": None,
                }
                try:
                    with path.open("r", encoding="utf-8") as fp:
                        data = json.load(fp)
                    rec = track_web_summary(Track.from_json(data))
                    rec["name"] = n[:-5]
                except Exception:
                    pass
                by_name[n[:-5]] = rec
        items = [by_name[k] for k in sorted(by_name)]
        return items

    def _list_slam_maps(self) -> list[dict]:
        folder = ROOT / "maps"
        items: list[dict] = []
        try:
            names = sorted(folder.iterdir(), key=lambda p: p.stat().st_mtime, reverse=True)
        except OSError:
            return items
        keep = {".simplemap", ".mm", ".json", ".png"}
        for path in names:
            if not path.is_file() or path.suffix.lower() not in keep:
                continue
            if path.name in ("qualified.json", "selected_map.local"):
                continue
            try:
                st = path.stat()
            except OSError:
                continue
            stem = path.stem.lower()
            usable = "archive"
            if path.suffix.lower() == ".json":
                usable = "occupancy"
            elif stem in ("lab", "lab2"):
                usable = "not_for_loc"
            elif stem in _qualified_stems():
                usable = "qualified"
            items.append({
                "name": path.name,
                "kind": path.suffix.lower().lstrip("."),
                "bytes": int(st.st_size),
                "mtime": int(st.st_mtime),
                "usable": usable,
            })
        return items

    def _probe_display(self) -> str | None:
        if os.environ.get("DISPLAY"):
            return os.environ["DISPLAY"]
        try:
            socks = sorted(n for n in os.listdir("/tmp/.X11-unix") if n.startswith("X"))
        except OSError:
            return None
        return f":{socks[0][1:]}" if socks else None

    def _launch_script(self, script: Path) -> bool:
        if not script.is_file():
            return False
        env = dict(os.environ)
        if not env.get("DISPLAY"):
            display = self._probe_display()
            if display:
                env["DISPLAY"] = display
                xa = os.path.expanduser("~/.Xauthority")
                if os.path.exists(xa):
                    env["XAUTHORITY"] = xa
        try:
            log_path = JETSON / ".local_viewer.log"
            logf = open(log_path, "a", encoding="utf-8")
            proc = subprocess.Popen(
                [sys.executable, str(script)],
                cwd=str(script.parent),
                env=env,
                stdout=logf,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self._viewers.append(proc)
            return True
        except Exception:
            return False

    def _close_viewers(self) -> bool:
        closed = False
        for proc in self._viewers:
            if proc.poll() is None:
                proc.terminate()
                closed = True
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
        self._viewers = [p for p in self._viewers if p.poll() is None]
        return closed

    def _open_lidar_view(self, mode: str) -> str:
        if mode == "off":
            return "雷达窗口已关闭" if self._close_viewers() else "没有正在跑的雷达窗口"
        try:
            from bunker_mini.lidar import MSOP_PORT, msop_udp_bound
        except Exception:
            msop_udp_bound = None  # type: ignore[assignment]
            MSOP_PORT = 6699
        if msop_udp_bound is not None and msop_udp_bound(MSOP_PORT):
            return (
                "MSOP 6699 已被占用（多半是 rslidar_sdk 建图）。"
                "未开工控机桌面窗。"
                "雷达画面在浏览器「雷达 / 地图」页：点「雷达开」订 /rslidar_points。"
            )
        opened = []
        if mode in ("cloud", "live", "map", "both"):
            if self._launch_script(JETSON / "view_lidar.py"):
                opened.append("view_lidar.py")
        if mode in ("both", "air"):
            air = JETSON / "live_airview.py"
            if not air.is_file():
                air = ROOT / "live_airview.py"
            if air.is_file() and self._launch_script(air):
                opened.append(air.name)
        if opened:
            return "已在工控机桌面打开 " + " + ".join(opened)
        return "未找到可启动的雷达窗口脚本"

    def _sanitize_cmd(self, msg: dict) -> dict | None:
        action = str(msg.get("action") or "")
        if action not in _ALLOWED_ACTIONS:
            return None
        out: dict = {"action": action, "ts": int(time.time() * 1000)}
        try:
            if action == "move":
                out["v"] = float(msg.get("v") or 0.0)
                out["w"] = float(msg.get("w") or 0.0)
                out["duration"] = max(0.0, float(msg.get("duration") or 0.0))
                if msg.get("openLoop"):
                    out["openLoop"] = True
            elif action == "goto":
                out["x"] = float(msg["x"])
                out["y"] = float(msg["y"])
                speed = float(msg.get("speed") or 0.0)
                if 0 < speed <= 0.5:
                    out["speed"] = speed
                if msg.get("yawDeg") is not None and str(msg.get("yawDeg")).strip() != "":
                    out["yawDeg"] = float(msg["yawDeg"])
            elif action == "set_wheelbase":
                out["wheelbaseM"] = float(msg["wheelbaseM"])
            elif action == "calibrate_wheelbase":
                out["yawDeg"] = float(msg["yawDeg"])
            elif action == "find_object":
                out["name"] = str(msg.get("name") or "target")
                out["approach"] = bool(msg.get("approach", True))
            elif action == "cancel":
                out["name"] = str(msg.get("name") or "target")
            elif action == "track_record":
                name = str(msg.get("name") or "").strip()
                if name:
                    out["name"] = name
            elif action in ("track_follow", "track_follow_back"):
                name = str(msg.get("trackId") or msg.get("name") or "").strip()
                if not name:
                    return None
                out["action"] = "track_follow"
                out["trackId"] = name
                out["bypassGuard"] = True
                if action == "track_follow_back":
                    self._roundtrip_track = name
                elif msg.get("reverse"):
                    out["reverse"] = True
            elif action == "track_delete":
                name = str(msg.get("trackId") or msg.get("name") or "").strip()
                if not name:
                    return None
                out["trackId"] = name
            elif action == "task_submit":
                name = str(msg.get("trackId") or msg.get("name") or "").strip()
                if not name:
                    return None
                out["trackId"] = name
                out["taskId"] = str(msg.get("taskId") or f"T{int(time.time() * 1000) % 100000}")
                out["from"] = ""
                out["to"] = ""
            elif action == "map_return":
                out["mapReturn"] = bool(msg.get("mapReturn", True))
            elif action == "pose_align":
                out["x"] = float(msg["x"])
                out["y"] = float(msg["y"])
                out["yawDeg"] = float(msg["yawDeg"])
            elif action == "map_upload":
                path = str(msg.get("file") or "").strip()
                if not path:
                    return None
                out["file"] = path
            elif action == "lidar_map":
                out["includeFree"] = bool(msg.get("includeFree", True))
                if msg.get("x") is not None and msg.get("y") is not None:
                    out["x"] = float(msg["x"])
                    out["y"] = float(msg["y"])
            elif action == "autonav_goto":
                out["x"] = float(msg["x"])
                out["y"] = float(msg["y"])
                speed = float(msg.get("speed") or 0.0)
                if 0 < speed <= 0.5:
                    out["speed"] = speed
                if msg.get("yawDeg") is not None and str(msg.get("yawDeg")).strip() != "":
                    out["yawDeg"] = float(msg["yawDeg"])
                name = str(msg.get("name") or msg.get("mapId") or "").strip()
                if name:
                    out["name"] = name
                frame = str(msg.get("frame") or "odom").strip().lower()
                if frame in ("map", "slam", "world"):
                    out["frame"] = "map"
                elif frame in ("sketch", "occ", "pseudo", "live"):
                    out["frame"] = "sketch"
                else:
                    out["frame"] = "odom"
            elif action == "autonav_select_map":
                name = str(msg.get("name") or msg.get("mapId") or "").strip()
                if not name:
                    return None
                out["name"] = name
            elif action in ("autonav_map", "autonav_cancel"):
                pass
        except (KeyError, TypeError, ValueError):
            return None
        return out

    def _push_chassis_now(self, conn: socket.socket | None = None) -> None:
        payload = self._pull_state()
        if payload is None:
            payload = {"chassis": {"heard": False}}
        msg = {"t": "state", "payload": payload}
        if conn is not None:
            self._ws_out(conn, msg)
        self._broadcast(msg)

    def _dispatch_cmd(self, payload: dict) -> None:
        try:
            tcp = self._ensure_tcp()
            if payload.get("action") == "estop":
                tcp.estop()
            else:
                tcp.cmd(payload)
            if str(payload.get("action") or "") == "query":
                deadline = time.monotonic() + 0.8
                while time.monotonic() < deadline:
                    with self._evt_lock:
                        events = tcp.poll_events()
                    got = False
                    for msg in events:
                        try:
                            self._on_tcp_msg(msg)
                        except Exception:
                            continue
                        if msg.get("type") == "state" or (
                            isinstance(msg.get("payload"), dict)
                            and msg.get("payload", {}).get("event") == "chassis"
                        ):
                            got = True
                    if got:
                        break
                    time.sleep(0.05)
            vis = str(payload.get("action") or "")
            if vis not in ("lidar_map", "point_cloud", "lidar_status", "autonav_map"):
                self._broadcast({"t": "cmd_ok", "action": vis})
        except Exception as exc:
            with self._tcp_lock:
                self._tcp = None
            self._broadcast({
                "t": "cmd_err",
                "action": str(payload.get("action") or ""),
                "msg": str(exc),
            })

    def _chassis_warn(self) -> str:
        now = time.monotonic()
        if now - self._warn_t < 1.0:
            return self._warn_cache
        self._warn_t = now
        warn = ""
        try:
            from bunker_mini.can_util import (
                list_socketcan_interfaces,
                socketcan_ctrl_state,
                socketcan_is_listen_only,
            )
            ifaces = list_socketcan_interfaces()
            listening = [n for n in ifaces if socketcan_is_listen_only(n)]
            bad = [
                f"{n}:{socketcan_ctrl_state(n)}"
                for n in ifaces
                if socketcan_ctrl_state(n) in ("error-passive", "bus-off")
            ]
            if listening:
                warn = (
                    "总线仍是 listen-only（" + ",".join(listening) +
                    "）。0x111 发不出去。请重启 run_local.py。"
                )
            elif bad:
                warn = (
                    "CAN 控制器异常（" + ",".join(bad) +
                    "）。网页已到工控机。"
                )
            else:
                try:
                    with open("/tmp/bunker_chassis_status", "r", encoding="utf-8") as fp:
                        info = json.load(fp)
                    if not info.get("heard"):
                        tx_fail = int(info.get("tx_fail") or 0)
                        tx_ok = int(info.get("tx_ok") or 0)
                        if tx_fail > 0 and tx_ok == 0:
                            warn = (
                                "底盘未锁定 0x211。不是频率问题："
                                "工控机发送队列已堵（ENOBUFS），按键帧发不出去。"
                                "请重启 run_local.py（已改为空闲不发 0x421）。"
                            )
                        else:
                            warn = (
                                "底盘未锁定 0x211（终端 mode=?）。"
                                "手册：上电后 0x211 每 200ms 广播；未开遥控须持续 0x421 才进指令模式。"
                                "网页→工控机正常。当前总线上没有底盘状态帧。"
                            )
                except Exception:
                    pass
        except Exception:
            pass
        self._warn_cache = warn
        return warn

    def _ensure_tcp(self) -> TeleopTcpClient:
        with self._tcp_lock:
            if self._tcp is not None:
                return self._tcp
            cli = TeleopTcpClient(self.tcp_host, self.tcp_port, self.token)
            cli.hello()
            self._tcp = cli
            return cli

    def _prune_sessions(self) -> None:
        now = time.monotonic()
        dead = [tok for tok, exp in self._sessions.items() if exp <= now]
        for tok in dead:
            self._sessions.pop(tok, None)

    def _authorized(self, headers: dict[str, str]) -> bool:
        sid = _cookie_value(headers.get("cookie", ""), _SID_COOKIE)
        if not sid:
            return False
        now = time.monotonic()
        with self._sess_lock:
            self._prune_sessions()
            exp = self._sessions.get(sid)
            if exp is None or exp <= now:
                return False
            self._sessions[sid] = now + _SESSION_TTL_S
            return True

    def _issue_session(self) -> str:
        sid = secrets.token_urlsafe(32)
        with self._sess_lock:
            self._prune_sessions()
            self._sessions[sid] = time.monotonic() + _SESSION_TTL_S
        return sid

    def _drop_session(self, headers: dict[str, str]) -> None:
        sid = _cookie_value(headers.get("cookie", ""), _SID_COOKIE)
        if not sid:
            return
        with self._sess_lock:
            self._sessions.pop(sid, None)

    def _password_ok(self, password: str) -> bool:
        digest = hashlib.sha256(password.encode("utf-8")).digest()
        return hmac.compare_digest(digest, self._password_digest)

    def _login_blocked(self, ip: str) -> bool:
        now = time.monotonic()
        with self._sess_lock:
            hits = [t for t in self._fail_at.get(ip, []) if now - t < _LOGIN_WINDOW_S]
            self._fail_at[ip] = hits
            return len(hits) >= _LOGIN_MAX_FAILS

    def _note_login_fail(self, ip: str) -> None:
        with self._sess_lock:
            self._fail_at.setdefault(ip, []).append(time.monotonic())

    def _clear_login_fail(self, ip: str) -> None:
        with self._sess_lock:
            self._fail_at.pop(ip, None)

    def _accept(self) -> None:
        while not self._stop.is_set() and self._sock is not None:
            try:
                conn, addr = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            peer = addr[0] if addr else ""
            threading.Thread(target=self._http_client, args=(conn, peer), daemon=True).start()

    def _http_client(self, conn: socket.socket, peer: str = "") -> None:
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.settimeout(5.0)
        try:
            req = _read_http(conn)
            if req is None:
                return
            method, path, headers, body = req
            if method == "POST" and path == "/login":
                self._handle_login(conn, peer, body)
                return
            if method in ("GET", "POST") and path == "/logout":
                self._drop_session(headers)
                _http_send(
                    conn, 303, b"",
                    extra=(
                        "Location: /\r\n"
                        f"Set-Cookie: {_SID_COOKIE}=; HttpOnly; Path=/; SameSite=Lax; Max-Age=0\r\n"
                    ),
                )
                return
            if headers.get("upgrade", "").lower() == "websocket" and path.startswith("/ws"):
                if not self._authorized(headers):
                    _http_send(conn, 401, _login_page_bytes("请先登录后再连接遥控通道"), ctype="text/html; charset=utf-8")
                    return
                key = headers.get("sec-websocket-key", "")
                if not key:
                    conn.sendall(b"HTTP/1.1 400 Bad Request\r\n\r\n")
                    return
                acc = _ws_accept(key)
                conn.sendall(
                    (
                        "HTTP/1.1 101 Switching Protocols\r\n"
                        "Upgrade: websocket\r\n"
                        "Connection: Upgrade\r\n"
                        f"Sec-WebSocket-Accept: {acc}\r\n"
                        "\r\n"
                    ).encode("ascii")
                )
                self._ws_loop(conn)
                return
            if path in ("/", "/index.html", "/login"):
                if self._authorized(headers) and path != "/login":
                    page = _page_bytes()
                    _http_send(conn, 200, page, ctype="text/html; charset=utf-8")
                else:
                    _http_send(conn, 401, _login_page_bytes(), ctype="text/html; charset=utf-8")
                return
            if path == "/health":
                _http_send(conn, 200, b"ok\n")
                return
            if not self._authorized(headers):
                _http_send(conn, 401, b"unauthorized\n")
                return
            if path in ("/chassis", "/chassis.json"):
                payload = self._pull_state() or {"chassis": {"heard": False}}
                raw = json.dumps({"t": "state", "payload": payload}, ensure_ascii=False).encode("utf-8")
                _http_send(conn, 200, raw, ctype="application/json; charset=utf-8")
                return
            if path in ("/teleop_from_laptop.py", "/dl/teleop_from_laptop.py"):
                src = ROOT / "teleop_from_laptop.py"
                if src.is_file():
                    raw = src.read_bytes()
                    _http_send(
                        conn, 200, raw,
                        ctype="text/plain; charset=utf-8",
                        extra='Content-Disposition: attachment; filename="teleop_from_laptop.py"\r\n',
                    )
                    return
            if path.startswith("/maps/"):
                name = unquote(path[len("/maps/"):])
                if (
                    not name
                    or name.startswith(".")
                    or "/" in name
                    or "\\" in name
                    or ".." in name
                ):
                    conn.sendall(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n")
                    return
                ext = Path(name).suffix.lower()
                if ext not in (".png", ".json"):
                    _http_send(conn, 403, b"only png or json\n")
                    return
                folder = (ROOT / "maps").resolve()
                src = (folder / name).resolve()
                try:
                    src.relative_to(folder)
                except ValueError:
                    conn.sendall(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n")
                    return
                if not src.is_file():
                    conn.sendall(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n")
                    return
                if src.stat().st_size > 8 * 1024 * 1024:
                    _http_send(conn, 413, b"too large\n")
                    return
                ctype = "image/png" if ext == ".png" else "application/json; charset=utf-8"
                _http_send(conn, 200, src.read_bytes(), ctype=ctype)
                return
            conn.sendall(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n")
        except OSError:
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def _handle_login(self, conn: socket.socket, peer: str, body: bytes) -> None:
        if self._login_blocked(peer or "unknown"):
            _http_send(
                conn, 429,
                _login_page_bytes("尝试次数过多，请稍后再试"),
                ctype="text/html; charset=utf-8",
            )
            return
        password = _form_password(body)
        if not self._password_ok(password):
            self._note_login_fail(peer or "unknown")
            _http_send(
                conn, 401,
                _login_page_bytes("密码错误，已拦截"),
                ctype="text/html; charset=utf-8",
            )
            return
        self._clear_login_fail(peer or "unknown")
        sid = self._issue_session()
        _http_send(
            conn, 303, b"",
            extra=(
                "Location: /\r\n"
                f"Set-Cookie: {_SID_COOKIE}={sid}; HttpOnly; Path=/; SameSite=Lax\r\n"
            ),
        )

    def _ws_loop(self, conn: socket.socket) -> None:
        self._clients += 1
        with self._ws_lock:
            self._ws_clients.append(conn)
        conn.settimeout(0.2)
        snap = self._last_state or self._state_from_file()
        if isinstance(snap, dict) and snap:
            self._ws_out(conn, {"t": "state", "payload": snap})
        try:
            while not self._stop.is_set():
                try:
                    frame = _ws_recv(conn)
                except _WsIdle:
                    continue
                except socket.timeout:
                    continue
                if frame is None:
                    break
                opcode, payload = frame
                if opcode == 0x8:
                    break
                if opcode == 0x9:
                    with self._out_lock:
                        _ws_send(conn, 0xA, payload)
                    continue
                if opcode not in (0x1, 0x2):
                    continue
                self._on_payload(conn, payload)
        finally:
            with self._ws_lock:
                if conn in self._ws_clients:
                    self._ws_clients.remove(conn)
            self._clients = max(0, self._clients - 1)
            if self._clients <= 0:
                try:
                    self._ensure_tcp().stick(0.0, 0.0)
                except Exception:
                    pass

    def _on_payload(self, conn: socket.socket, payload: bytes) -> None:
        try:
            msg = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError):
            return
        if not isinstance(msg, dict):
            return
        kind = str(msg.get("t") or "")
        try:
            tcp = self._ensure_tcp()
        except Exception:
            return
        if kind == "p":
            t0 = msg.get("t0")
            t1 = time.monotonic() * 1000.0
            self._ws_out(conn, {"t": "p", "t0": t0, "t1": t1})
            return
        if kind == "e":
            try:
                tcp.estop()
            except Exception:
                pass
            return
        if kind == "tracks":
            self._ws_out(conn, {"t": "tracks", "items": self._list_tracks()})
            return
        if kind == "slam_maps":
            self._ws_out(conn, {"t": "slam_maps", "items": self._list_slam_maps()})
            return
        if kind == "chassis":
            threading.Thread(
                target=self._push_chassis_now, args=(conn,),
                name="teleop-web-chassis", daemon=True,
            ).start()
            return
        if kind == "view":
            mode = str(msg.get("mode") or "cloud")
            text = self._open_lidar_view(mode)
            self._ws_out(conn, {"t": "view", "mode": mode, "msg": text})
            return
        if kind == "c":
            payload = self._sanitize_cmd(msg)
            if payload is None:
                self._ws_out(conn, {
                    "t": "cmd_err",
                    "action": str(msg.get("action") or ""),
                    "msg": "参数无效或不支持的指令",
                })
                return
            threading.Thread(
                target=self._dispatch_cmd, args=(payload,),
                name="teleop-web-cmd", daemon=True,
            ).start()
            return
        if kind == "s":
            try:
                v = float(msg.get("v") or 0.0)
                w = float(msg.get("w") or 0.0)
            except (TypeError, ValueError):
                return
            v = max(-2.0, min(2.0, v))
            w = max(-4.0, min(4.0, w))
            try:
                tcp.stick(v, w)
            except Exception:
                with self._tcp_lock:
                    self._tcp = None
                return
            # 100Hz stick 每帧 ACK + 读 CAN 状态会堵 WS，空档超过看门狗
            # 就变成角速度一卡一卡。只在杆量变化或 5Hz 时回一次。
            now = time.monotonic()
            changed = self._last_ack_vw != (v, w)
            if changed or now - self._last_ack_t >= 0.2:
                self._last_ack_vw = (v, w)
                self._last_ack_t = now
                warn = self._chassis_warn()
                self._ws_out(conn, {"t": "ack", "v": v, "w": w, "warn": warn})


def _http_send(
    conn: socket.socket,
    status: int,
    body: bytes,
    *,
    ctype: str = "text/plain; charset=utf-8",
    extra: str = "",
) -> None:
    reason = {
        200: "OK",
        303: "See Other",
        401: "Unauthorized",
        403: "Forbidden",
        404: "Not Found",
        413: "Payload Too Large",
        429: "Too Many Requests",
    }.get(status, "OK")
    conn.sendall(
        (
            f"HTTP/1.1 {status} {reason}\r\n"
            f"Content-Type: {ctype}\r\n"
            f"Content-Length: {len(body)}\r\n"
            "Cache-Control: no-store\r\n"
            f"{extra}"
            "Connection: close\r\n"
            "\r\n"
        ).encode("ascii", "replace") + body
    )


def _cookie_value(raw: str, name: str) -> str:
    for part in raw.split(";"):
        item = part.strip()
        if not item:
            continue
        key, _, val = item.partition("=")
        if key.strip() == name:
            return val.strip()
    return ""


def _form_password(body: bytes) -> str:
    text = body.decode("utf-8", "replace")
    if text.lstrip().startswith("{"):
        try:
            obj = json.loads(text)
        except json.JSONDecodeError:
            obj = {}
        if isinstance(obj, dict):
            return str(obj.get("password") or "")
    parsed = parse_qs(text, keep_blank_values=True)
    vals = parsed.get("password") or []
    if vals:
        return vals[0]
    return ""


def _login_page_bytes(error: str = "") -> bytes:
    err = ""
    if error:
        esc = (
            error.replace("&", "&amp;")
            .replace("<", "&lt;")
            .replace(">", "&gt;")
        )
        err = f'<p class="err">{esc}</p>'
    html = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>Bunker 控制台登录</title>
<style>
  html, body {{
    margin: 0; min-height: 100%;
    background: radial-gradient(900px 420px at 20% -10%, #163044 0%, transparent 55%), #0a1016;
    color: #e8eef5;
    font: 15px/1.45 "Segoe UI", "PingFang SC", "Noto Sans SC", "Microsoft YaHei", sans-serif;
  }}
  .box {{
    max-width: 420px; margin: 12vh auto; padding: 28px 26px;
    background: #141d27; border: 1px solid #243140; border-radius: 16px;
  }}
  h1 {{ margin: 0 0 8px; font-size: 20px; letter-spacing: .04em; }}
  p {{ color: #8b9aab; font-size: 13px; }}
  .err {{ color: #ff4d5a; }}
  label {{ display: block; margin: 16px 0 6px; color: #8b9aab; font-size: 12px; }}
  input {{
    width: 100%; box-sizing: border-box; padding: 10px 12px;
    background: #0d141c; border: 1px solid #334556; border-radius: 8px;
    color: #e8eef5; font: inherit;
  }}
  button {{
    margin-top: 16px; width: 100%; padding: 10px 12px; border-radius: 8px;
    border: 1px solid #2d6f8f; background: #123346; color: #d7f3ff;
    font: inherit; cursor: pointer;
  }}
</style>
</head>
<body>
  <div class="box">
    <h1>BUNKER 控制台</h1>
    <p>首次进入必须输入正确密码，否则拦截控制台、遥控通道和底盘接口。</p>
    {err}
    <form method="POST" action="/login" autocomplete="on">
      <label for="password">控制台密码</label>
      <input id="password" name="password" type="password" required autofocus autocomplete="current-password"/>
      <button type="submit">进入控制台</button>
    </form>
  </div>
</body>
</html>
"""
    return html.encode("utf-8")


def _read_http(conn: socket.socket) -> tuple[str, str, dict[str, str], bytes] | None:
    data = bytearray()
    while b"\r\n\r\n" not in data and len(data) < 16384:
        chunk = conn.recv(512)
        if not chunk:
            return None
        data.extend(chunk)
    raw = bytes(data)
    head, sep, rest = raw.partition(b"\r\n\r\n")
    if not sep:
        return None
    lines = head.decode("iso-8859-1", "replace").split("\r\n")
    if not lines:
        return None
    parts = lines[0].split()
    method = parts[0].upper() if parts else "GET"
    path = parts[1] if len(parts) >= 2 else "/"
    path = path.split("?", 1)[0]
    headers: dict[str, str] = {}
    for line in lines[1:]:
        if ":" not in line:
            continue
        k, _, v = line.partition(":")
        headers[k.strip().lower()] = v.strip()
    try:
        clen = int(headers.get("content-length") or 0)
    except ValueError:
        clen = 0
    if clen < 0 or clen > 4096:
        return None
    body = rest
    while len(body) < clen:
        chunk = conn.recv(clen - len(body))
        if not chunk:
            return None
        body += chunk
    return method, path, headers, body[:clen]


def main() -> int:
    parser = argparse.ArgumentParser(description="工控机网页遥控（浏览器 HID → WS → :9100）")
    parser.add_argument("--tcp-host", default="127.0.0.1")
    parser.add_argument("--tcp-port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--token", default=DEFAULT_TOKEN)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=DEFAULT_WEB_PORT)
    args = parser.parse_args()
    try:
        srv = TeleopWebServer(
            tcp_host=args.tcp_host,
            tcp_port=args.tcp_port,
            token=args.token,
            http_host=args.bind,
            http_port=args.port,
        )
        srv.start()
    except Exception as exc:
        print(f"启动失败（先开 python3 run_local.py）: {exc}", file=sys.stderr)
        return 1
    urls = "\n  ".join(srv.urls())
    print("网页控制台已开。笔记本浏览器打开（WASD 遥控 + 任务按钮）：")
    print(f"  {urls}")
    print("Ctrl+C 退出")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n已停")
    finally:
        srv.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
