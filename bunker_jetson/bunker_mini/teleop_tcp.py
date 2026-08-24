"""Low-latency binary TCP teleop (not SSH, not WebSocket JSON).

Laptop HID / operator PC  --TCP 100 Hz-->  Jetson :9100  --> set_velocity_now
                                                              CAN TX 100 Hz

Frame (little-endian, 12-byte header)::

    u16 magic=0xB1CA
    u8  type
    u8  flags
    u16 seq
    u32 t_us     client clock, for RTT
    u16 plen
    bytes payload
"""

from __future__ import annotations

import json
import os
import socket
import struct
import threading
import time
from collections import deque
from typing import Callable, Optional

MAGIC = 0xB1CA
HEADER = struct.Struct("<HBBHIH")
HEADER_SIZE = HEADER.size  # 12
MAX_PAYLOAD = 256
# plen 是 u16；stick/move 仍建议 ≤256，命令/事件 JSON 可到 60KB
MAX_FRAME = 60_000

TYPE_HELLO = 1
TYPE_PING = 2
TYPE_STICK = 3
TYPE_MOVE = 4
TYPE_ESTOP = 5
TYPE_QUERY = 6
TYPE_GOTO = 7
TYPE_CMD = 8          # JSON 任务指令（find_object / track_record / lidar_on …）
TYPE_ACK = 0x81
TYPE_STATE = 0x82
TYPE_EVENT = 0x83     # JSON 状态/事件回传（不走 WebSocket）
FLAG_ACK = 0x01       # 请求应答；stick 默认不置，避免 100Hz 占满 Wi-Fi ACK

STICK = struct.Struct("<hhB")  # v mm/s, w mrad/s, buttons
MOVE = struct.Struct("<fffB")  # v, w, duration, flags
GOTO = struct.Struct("<fff")   # x, y, speed
STATE = struct.Struct("<ff")   # linear, angular

BTN_SPACE = 0x01
BTN_Q = 0x02
BTN_PLUS = 0x04
BTN_MINUS = 0x08
BTN_B = 0x10
MOVE_BYPASS = 0x01

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 9100
DEFAULT_TOKEN = "bunker-teleop"
# 丢流急停，不是猜键。HID/网页松键靠立刻发 (0,0)；这里只防 Wi-Fi/进程挂掉。
# 丢流才停。HID/网页松键靠立刻发 (0,0)；TX 环 100Hz 会自己保活最后一帧。
# 160ms 太紧：Wi-Fi 或客户端 drain 事件卡一拍，看门狗 stop_motion，
# 下一帧又从 0 拉角速度——长按 A/D 一卡一卡。0.40s 仍低于底盘 500ms 无帧超时。
STICK_WATCHDOG_S = 0.40
HELLO_TIMEOUT_S = 2.0


def now_us() -> int:
    return int(time.monotonic() * 1_000_000) & 0xFFFFFFFF


def pack_frame(typ: int, payload: bytes = b"", seq: int = 0, t_us: int = 0,
               flags: int = 0) -> bytes:
    if len(payload) > MAX_FRAME:
        raise ValueError("payload too long")
    return HEADER.pack(MAGIC, typ, flags, seq & 0xFFFF, t_us & 0xFFFFFFFF,
                       len(payload)) + payload


def try_parse(buf: bytearray) -> Optional[tuple[int, int, int, int, bytes]]:
    """Return (type, flags, seq, t_us, payload) or None if incomplete."""
    if len(buf) < HEADER_SIZE:
        return None
    magic, typ, flags, seq, t_us, plen = HEADER.unpack_from(buf, 0)
    if magic != MAGIC or plen > MAX_FRAME:
        del buf[:1]
        return None
    if len(buf) < HEADER_SIZE + plen:
        return None
    payload = bytes(buf[HEADER_SIZE:HEADER_SIZE + plen])
    del buf[: HEADER_SIZE + plen]
    return typ, flags, seq, t_us, payload


def recv_frame(sock: socket.socket, timeout: float) -> Optional[tuple]:
    sock.settimeout(timeout)
    buf = bytearray()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            chunk = sock.recv(512)
        except socket.timeout:
            return None
        if not chunk:
            return None
        buf.extend(chunk)
        parsed = try_parse(buf)
        if parsed is not None:
            return parsed
    return None


class TeleopTcpServer:
    """Accept one operator connection; apply stick/move on the calling thread pool."""

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        token: str = DEFAULT_TOKEN,
        *,
        on_stick: Optional[Callable[[float, float], None]] = None,
        on_estop: Optional[Callable[[], None]] = None,
        on_move: Optional[Callable[[float, float, float, bool], None]] = None,
        on_goto: Optional[Callable[[float, float, float], None]] = None,
        on_query: Optional[Callable[[], tuple[float, float]]] = None,
        on_idle_stop: Optional[Callable[[], None]] = None,
        on_cmd: Optional[Callable[[dict], None]] = None,
        on_rx: Optional[Callable[[], None]] = None,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.token = token or DEFAULT_TOKEN
        self.on_stick = on_stick
        self.on_estop = on_estop
        self.on_move = on_move
        self.on_goto = on_goto
        self.on_query = on_query
        self.on_idle_stop = on_idle_stop
        self.on_cmd = on_cmd
        self.on_rx = on_rx
        self._sock: Optional[socket.socket] = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_stick_t = 0.0
        self._watchdog: Optional[threading.Thread] = None
        self._conns: list[socket.socket] = []
        self._conns_lock = threading.Lock()
        self.clients = 0
        self.last_rtt_us = 0

    def stick_age_s(self) -> float:
        """Seconds since the last stick frame, or inf if none this session."""
        t = self._last_stick_t
        if t <= 0.0:
            return float("inf")
        return time.monotonic() - t

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        srv.bind((self.host, self.port))
        srv.listen(2)
        srv.settimeout(0.5)
        self._sock = srv
        self._thread = threading.Thread(target=self._accept_loop, name="teleop-tcp", daemon=True)
        self._thread.start()
        self._watchdog = threading.Thread(target=self._watchdog_loop, name="teleop-wd", daemon=True)
        self._watchdog.start()

    def stop(self) -> None:
        self._stop.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._watchdog:
            self._watchdog.join(timeout=1.0)
        with self._conns_lock:
            for conn in list(self._conns):
                try:
                    conn.close()
                except OSError:
                    pass
            self._conns.clear()

    def push_event(self, msg: dict) -> None:
        """把状态/事件推给已鉴权的 TCP 控制台（不经 WebSocket）。"""
        try:
            raw = json.dumps(msg, ensure_ascii=False).encode("utf-8")
        except (TypeError, ValueError):
            return
        if len(raw) > MAX_FRAME:
            return
        frame = pack_frame(TYPE_EVENT, raw)
        with self._conns_lock:
            conns = list(self._conns)
        for conn in conns:
            try:
                conn.sendall(frame)
            except OSError:
                pass

    def _accept_loop(self) -> None:
        while not self._stop.is_set() and self._sock is not None:
            try:
                conn, addr = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(
                target=self._client_loop, args=(conn, addr),
                name="teleop-cli", daemon=True,
            ).start()

    def _watchdog_loop(self) -> None:
        while not self._stop.is_set():
            time.sleep(0.04)
            if self.clients <= 0 or self._last_stick_t <= 0:
                continue
            if time.monotonic() - self._last_stick_t > STICK_WATCHDOG_S:
                self._last_stick_t = 0.0
                stop = self.on_idle_stop or self.on_estop
                if stop is not None:
                    try:
                        stop()
                    except Exception:
                        pass

    def _client_loop(self, conn: socket.socket, addr) -> None:
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        conn.settimeout(0.2)
        self.clients += 1
        buf = bytearray()
        authed = False
        try:
            hello = recv_frame(conn, HELLO_TIMEOUT_S)
            if hello is None or hello[0] != TYPE_HELLO:
                return
            _typ, _flags, seq, t_us, payload = hello
            token = payload.decode("utf-8", "replace")
            if token != self.token:
                conn.sendall(pack_frame(TYPE_ACK, b"\x01", seq=seq, t_us=t_us))
                return
            authed = True
            with self._conns_lock:
                self._conns.append(conn)
            conn.sendall(pack_frame(TYPE_ACK, b"\x00", seq=seq, t_us=t_us))
            while not self._stop.is_set():
                try:
                    chunk = conn.recv(512)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buf.extend(chunk)
                while True:
                    parsed = try_parse(buf)
                    if parsed is None:
                        break
                    self._handle(conn, parsed)
        except OSError:
            pass
        finally:
            self.clients = max(0, self.clients - 1)
            with self._conns_lock:
                if conn in self._conns:
                    self._conns.remove(conn)
            if authed:
                stop = self.on_idle_stop or self.on_estop
                if stop is not None:
                    try:
                        stop()
                    except Exception:
                        pass
            try:
                conn.close()
            except OSError:
                pass

    def _handle(self, conn: socket.socket, parsed: tuple) -> None:
        typ, flags, seq, t_us, payload = parsed
        if self.on_rx is not None:
            try:
                self.on_rx()
            except Exception:
                pass
        if typ == TYPE_PING:
            conn.sendall(pack_frame(TYPE_ACK, b"\x00", seq=seq, t_us=t_us))
            return
        if typ == TYPE_STICK and len(payload) >= STICK.size:
            v_mm, w_mrad, _btns = STICK.unpack_from(payload)
            self._last_stick_t = time.monotonic()
            if self.on_stick is not None:
                self.on_stick(v_mm / 1000.0, w_mrad / 1000.0)
            # 默认不 ACK：100Hz stick 的回程会占 Wi-Fi，客户端也不等。
            if flags & FLAG_ACK:
                conn.sendall(pack_frame(TYPE_ACK, b"\x00", seq=seq, t_us=t_us))
            return
        if typ == TYPE_MOVE and len(payload) >= MOVE.size:
            v, w, dur, flags = MOVE.unpack_from(payload)
            if self.on_move is not None:
                self.on_move(v, w, dur, bool(flags & MOVE_BYPASS))
            conn.sendall(pack_frame(TYPE_ACK, b"\x00", seq=seq, t_us=t_us))
            return
        if typ == TYPE_ESTOP:
            if self.on_estop is not None:
                self.on_estop()
            conn.sendall(pack_frame(TYPE_ACK, b"\x00", seq=seq, t_us=t_us))
            return
        if typ == TYPE_QUERY:
            v, w = (0.0, 0.0)
            if self.on_query is not None:
                v, w = self.on_query()
            conn.sendall(pack_frame(TYPE_STATE, STATE.pack(v, w), seq=seq, t_us=t_us))
            return
        if typ == TYPE_GOTO and len(payload) >= GOTO.size:
            x, y, speed = GOTO.unpack_from(payload)
            if self.on_goto is not None:
                self.on_goto(x, y, speed)
            conn.sendall(pack_frame(TYPE_ACK, b"\x00", seq=seq, t_us=t_us))
            return
        if typ == TYPE_CMD:
            try:
                obj = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                conn.sendall(pack_frame(TYPE_ACK, b"\x03", seq=seq, t_us=t_us))
                return
            if not isinstance(obj, dict):
                conn.sendall(pack_frame(TYPE_ACK, b"\x03", seq=seq, t_us=t_us))
                return
            # 先 ACK 再异步执行：map_upload / 回放等慢命令不能挡住 100Hz stick
            conn.sendall(pack_frame(TYPE_ACK, b"\x00", seq=seq, t_us=t_us))
            if self.on_cmd is not None:
                threading.Thread(
                    target=self._run_cmd, args=(obj,), name="teleop-cmd", daemon=True,
                ).start()
            return
        conn.sendall(pack_frame(TYPE_ACK, b"\x02", seq=seq, t_us=t_us))

    def _run_cmd(self, obj: dict) -> None:
        try:
            self.on_cmd(obj)
        except Exception:
            pass


class TeleopTcpClient:
    """Operator-side socket. Call hello() then ping/stick/move.

    收包在独立线程：``cmd`` / ``estop`` 等 ACK 不会挡住 100Hz ``stick``。
    """

    def __init__(self, host: str, port: int = DEFAULT_PORT, token: str = DEFAULT_TOKEN) -> None:
        self.host = host
        self.port = int(port)
        self.token = token
        self.sock = socket.create_connection((host, port), timeout=5.0)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._seq = 0
        self._seq_lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._events: deque = deque(maxlen=200)
        self._acks: dict[int, tuple] = {}
        self._waiting: set[int] = set()
        self._ack_cv = threading.Condition()
        self._closed = False
        self._reader = threading.Thread(target=self._read_loop, name="teleop-rx", daemon=True)
        self._reader.start()

    def close(self) -> None:
        self._closed = True
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass
        if self._reader.is_alive():
            self._reader.join(timeout=1.0)
        with self._ack_cv:
            self._ack_cv.notify_all()

    def _next_seq(self) -> int:
        with self._seq_lock:
            self._seq = (self._seq + 1) & 0xFFFF
            return self._seq

    def _read_loop(self) -> None:
        buf = bytearray()
        while not self._closed:
            try:
                self.sock.settimeout(0.2)
                chunk = self.sock.recv(512)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf.extend(chunk)
            while True:
                parsed = try_parse(buf)
                if parsed is None:
                    break
                rtyp, _flags, rseq, t_us, rpay = parsed
                if rtyp == TYPE_EVENT:
                    try:
                        self._events.append(json.loads(rpay.decode("utf-8")))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        pass
                    continue
                with self._ack_cv:
                    if rseq in self._waiting:
                        self._acks[rseq] = (rtyp, rpay, t_us)
                        self._ack_cv.notify_all()

    def _request(self, typ: int, payload: bytes = b"", timeout: float = 0.4,
                 flags: int = 0) -> tuple:
        seq = self._next_seq()
        t0 = now_us()
        with self._ack_cv:
            self._waiting.add(seq)
            self._acks.pop(seq, None)
        try:
            with self._send_lock:
                if self._closed:
                    raise OSError("teleop TCP 已关闭")
                self.sock.sendall(pack_frame(typ, payload, seq=seq, t_us=t0, flags=flags))
            deadline = time.monotonic() + timeout
            with self._ack_cv:
                while time.monotonic() < deadline:
                    got = self._acks.pop(seq, None)
                    if got is not None:
                        rtyp, rpay, t_us = got
                        rtt_us = (now_us() - t_us) & 0xFFFFFFFF
                        if rtt_us > 2_000_000:
                            rtt_us = 0
                        return rtyp, rpay, rtt_us
                    remain = deadline - time.monotonic()
                    if remain <= 0:
                        break
                    self._ack_cv.wait(timeout=remain)
        finally:
            with self._ack_cv:
                self._waiting.discard(seq)
                self._acks.pop(seq, None)
        raise TimeoutError("teleop TCP 无应答")

    def hello(self) -> int:
        _typ, payload, rtt = self._request(TYPE_HELLO, self.token.encode("utf-8"))
        if payload[:1] != b"\x00":
            raise PermissionError("teleop token 不匹配")
        return rtt

    def ping(self) -> int:
        _typ, _pay, rtt = self._request(TYPE_PING)
        return rtt

    def drain(self) -> int:
        """兼容旧调用：收包已在读线程，这里不再抢 socket。"""
        return 0

    def poll_events(self) -> list[dict]:
        """非阻塞取出已收到的状态/事件。"""
        out = list(self._events)
        self._events.clear()
        return out

    def cmd(self, payload: dict, timeout: float = 3.0) -> int:
        """下发一条与云端同语义的 JSON 任务指令。"""
        raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        if len(raw) > MAX_FRAME:
            raise ValueError("cmd payload too long")
        _typ, _pay, rtt = self._request(TYPE_CMD, raw, timeout=timeout)
        return rtt

    def stick(self, v: float, w: float, buttons: int = 0, wait_ack: bool = False) -> int:
        """默认只发不等：Wi-Fi RTT / 任务 ACK 都不会卡住 100Hz。"""
        v_mm = int(max(-32000, min(32000, round(v * 1000))))
        w_mrad = int(max(-32000, min(32000, round(w * 1000))))
        payload = STICK.pack(v_mm, w_mrad, buttons)
        if wait_ack:
            _typ, _pay, rtt = self._request(TYPE_STICK, payload, timeout=0.15, flags=FLAG_ACK)
            return rtt
        seq = self._next_seq()
        with self._send_lock:
            if self._closed:
                raise OSError("teleop TCP 已关闭")
            self.sock.sendall(pack_frame(TYPE_STICK, payload, seq=seq, t_us=now_us()))
        return 0

    def move(self, v: float, w: float, duration: float = 0.0, bypass: bool = True) -> int:
        flags = MOVE_BYPASS if bypass else 0
        _typ, _pay, rtt = self._request(TYPE_MOVE, MOVE.pack(v, w, duration, flags))
        return rtt

    def estop(self) -> int:
        _typ, _pay, rtt = self._request(TYPE_ESTOP)
        return rtt

    def query(self) -> tuple[float, float, int]:
        typ, payload, rtt = self._request(TYPE_QUERY)
        if typ != TYPE_STATE or len(payload) < STATE.size:
            return 0.0, 0.0, rtt
        v, w = STATE.unpack_from(payload)
        return v, w, rtt

    def goto(self, x: float, y: float, speed: float = 0.0) -> int:
        _typ, _pay, rtt = self._request(TYPE_GOTO, GOTO.pack(x, y, speed))
        return rtt


def server_from_env(**kwargs) -> TeleopTcpServer:
    enabled = os.environ.get("BUNKER_TELEOP_TCP", "1").lower() not in ("0", "false", "no")
    if not enabled:
        raise RuntimeError("BUNKER_TELEOP_TCP=0")
    return TeleopTcpServer(
        host=os.environ.get("BUNKER_TELEOP_HOST", DEFAULT_HOST),
        port=int(os.environ.get("BUNKER_TELEOP_PORT", str(DEFAULT_PORT))),
        token=os.environ.get("BUNKER_TELEOP_TOKEN", DEFAULT_TOKEN),
        **kwargs,
    )
