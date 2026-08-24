"""网页遥控：浏览器 HID 经 WS 转发到 TCP stick。"""

from __future__ import annotations

import json
import socket
import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
if str(_ROOT / "bunker_jetson") not in sys.path:
    sys.path.insert(0, str(_ROOT / "bunker_jetson"))

from bunker_mini.teleop_tcp import TeleopTcpServer  # noqa: E402
from teleop_web import TeleopWebServer, _ws_accept  # noqa: E402

WEB_PASSWORD = "fafu123456"


def _http(host: str, port: int, raw: bytes, timeout: float = 2.0) -> bytes:
    sock = socket.create_connection((host, port), timeout=timeout)
    try:
        sock.sendall(raw)
        out = b""
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            out += chunk
        return out
    finally:
        sock.close()


def _cookie_sid(resp: bytes) -> str:
    head = resp.split(b"\r\n\r\n", 1)[0].decode("iso-8859-1", "replace")
    for line in head.split("\r\n"):
        if line.lower().startswith("set-cookie:") and "bunker_sid=" in line:
            part = line.split(":", 1)[1].strip().split(";", 1)[0]
            return part.split("=", 1)[1].strip()
    return ""


def _login(host: str, port: int, password: str = WEB_PASSWORD) -> str:
    body = f"password={password}".encode("ascii")
    resp = _http(host, port, (
        b"POST /login HTTP/1.1\r\n"
        + f"Host: {host}\r\n".encode("ascii")
        + b"Content-Type: application/x-www-form-urlencoded\r\n"
        + f"Content-Length: {len(body)}\r\n".encode("ascii")
        + b"Connection: close\r\n\r\n"
        + body
    ))
    sid = _cookie_sid(resp)
    assert sid, f"登录未发会话: {resp[:400]!r}"
    assert b"303" in resp.split(b"\r\n", 1)[0]
    return sid


def _ws_client(host: str, port: int, sid: str | None = None) -> socket.socket:
    sock = socket.create_connection((host, port), timeout=2.0)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    key = "dGhlIHNhbXBsZSBub25jZQ=="
    cookie = f"Cookie: bunker_sid={sid}\r\n" if sid else ""
    req = (
        "GET /ws HTTP/1.1\r\n"
        f"Host: {host}:{port}\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"{cookie}"
        f"Sec-WebSocket-Key: {key}\r\n"
        "Sec-WebSocket-Version: 13\r\n"
        "\r\n"
    )
    sock.sendall(req.encode("ascii"))
    buf = bytearray()
    while b"\r\n\r\n" not in buf:
        chunk = sock.recv(512)
        assert chunk, "无 HTTP 升级应答"
        buf.extend(chunk)
    head = bytes(buf).split(b"\r\n\r\n", 1)[0].decode("iso-8859-1")
    assert "101" in head.split("\r\n", 1)[0]
    acc = ""
    for line in head.split("\r\n"):
        if line.lower().startswith("sec-websocket-accept:"):
            acc = line.split(":", 1)[1].strip()
    assert acc == _ws_accept(key)
    return sock


def _ws_send_text(sock: socket.socket, text: str) -> None:
    payload = text.encode("utf-8")
    mask = b"\x01\x02\x03\x04"
    header = bytes([0x81, 0x80 | len(payload)]) + mask
    masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    sock.sendall(header + masked)


def test_unauth_is_blocked():
    tcp = TeleopTcpServer(host="127.0.0.1", port=19117, token="web-test")
    tcp.start()
    time.sleep(0.05)
    web = TeleopWebServer(
        tcp_host="127.0.0.1", tcp_port=19117, token="web-test",
        http_host="127.0.0.1", http_port=19118,
    )
    try:
        web.start()
        time.sleep(0.05)
        page = _http("127.0.0.1", 19118, b"GET / HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n")
        assert b"401" in page.split(b"\r\n", 1)[0]
        assert "控制台密码".encode("utf-8") in page
        assert b"keydown" not in page
        assert "急停".encode("utf-8") not in page
        chassis = _http("127.0.0.1", 19118, b"GET /chassis.json HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n")
        assert b"401" in chassis.split(b"\r\n", 1)[0]
        bad = _http("127.0.0.1", 19118, (
            b"POST /login HTTP/1.1\r\nHost: 127.0.0.1\r\n"
            b"Content-Type: application/x-www-form-urlencoded\r\n"
            b"Content-Length: 16\r\nConnection: close\r\n\r\n"
            b"password=wrongpw"
        ))
        assert b"401" in bad.split(b"\r\n", 1)[0]
        assert _cookie_sid(bad) == ""
        sock = socket.create_connection(("127.0.0.1", 19118), timeout=2.0)
        sock.sendall(
            b"GET /ws HTTP/1.1\r\nHost: 127.0.0.1\r\n"
            b"Upgrade: websocket\r\nConnection: Upgrade\r\n"
            b"Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
            b"Sec-WebSocket-Version: 13\r\n\r\n"
        )
        raw = b""
        while b"\r\n\r\n" not in raw:
            chunk = sock.recv(512)
            assert chunk
            raw += chunk
        sock.close()
        assert b"101" not in raw.split(b"\r\n", 1)[0]
        assert b"401" in raw.split(b"\r\n", 1)[0]
    finally:
        web.stop()
        tcp.stop()


def test_http_page_and_ws_stick():
    sticks: list[tuple[float, float]] = []
    tcp = TeleopTcpServer(
        host="127.0.0.1",
        port=19111,
        token="web-test",
        on_stick=lambda v, w: sticks.append((v, w)),
    )
    tcp.start()
    time.sleep(0.05)
    web = TeleopWebServer(
        tcp_host="127.0.0.1",
        tcp_port=19111,
        token="web-test",
        http_host="127.0.0.1",
        http_port=19112,
    )
    try:
        web.start()
        time.sleep(0.05)
        sid = _login("127.0.0.1", 19112)
        page = _http("127.0.0.1", 19112, (
            b"GET / HTTP/1.1\r\nHost: 127.0.0.1\r\n"
            + f"Cookie: bunker_sid={sid}\r\n".encode("ascii")
            + b"Connection: close\r\n\r\n"
        ))
        assert b"200 OK" in page
        assert b"keydown" in page
        assert "find_object".encode() in page
        assert "探路".encode("utf-8") in page
        assert "底盘状态".encode("utf-8") in page
        assert "开环 move".encode("utf-8") in page
        assert "导航 goto".encode("utf-8") in page
        assert "功能总览".encode("utf-8") in page
        assert "#/nav".encode("ascii") in page

        ws = _ws_client("127.0.0.1", 19112, sid)
        _ws_send_text(ws, json.dumps({"t": "s", "v": 0.10, "w": 0.20}))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if any(abs(v - 0.10) < 1e-3 and abs(w - 0.20) < 1e-3 for v, w in sticks):
                break
            time.sleep(0.02)
        else:
            raise AssertionError(f"未收到 stick: {sticks}")
        ws.close()
    finally:
        web.stop()
        tcp.stop()


def test_ws_cmd_find_object():
    cmds: list[dict] = []
    tcp = TeleopTcpServer(
        host="127.0.0.1",
        port=19113,
        token="web-test",
        on_cmd=lambda p: cmds.append(p),
    )
    tcp.start()
    time.sleep(0.05)
    web = TeleopWebServer(
        tcp_host="127.0.0.1",
        tcp_port=19113,
        token="web-test",
        http_host="127.0.0.1",
        http_port=19114,
    )
    try:
        web.start()
        time.sleep(0.05)
        sid = _login("127.0.0.1", 19114)
        ws = _ws_client("127.0.0.1", 19114, sid)
        _ws_send_text(ws, json.dumps({
            "t": "c", "action": "find_object", "name": "A", "approach": False,
        }))
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            if any(c.get("action") == "find_object" and c.get("name") == "A" for c in cmds):
                break
            time.sleep(0.02)
        else:
            raise AssertionError(f"未收到 find_object: {cmds}")
        ws.close()
    finally:
        web.stop()
        tcp.stop()


def test_ws_cmd_move_open_loop():
    cmds: list[dict] = []
    tcp = TeleopTcpServer(
        host="127.0.0.1",
        port=19115,
        token="web-test",
        on_cmd=lambda p: cmds.append(p),
    )
    tcp.start()
    time.sleep(0.05)
    web = TeleopWebServer(
        tcp_host="127.0.0.1",
        tcp_port=19115,
        token="web-test",
        http_host="127.0.0.1",
        http_port=19116,
    )
    try:
        web.start()
        time.sleep(0.05)
        sid = _login("127.0.0.1", 19116)
        ws = _ws_client("127.0.0.1", 19116, sid)
        _ws_send_text(ws, json.dumps({
            "t": "c", "action": "move", "v": 0.15, "w": 0.0,
            "duration": 2.0, "openLoop": True,
        }))
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            hit = [
                c for c in cmds
                if c.get("action") == "move" and c.get("openLoop") is True
            ]
            if hit:
                assert abs(float(hit[0]["v"]) - 0.15) < 1e-6
                assert abs(float(hit[0]["duration"]) - 2.0) < 1e-6
                break
            time.sleep(0.02)
        else:
            raise AssertionError(f"未收到开环 move: {cmds}")
        ws.close()
    finally:
        web.stop()
        tcp.stop()


if __name__ == "__main__":
    test_unauth_is_blocked()
    test_http_page_and_ws_stick()
    test_ws_cmd_find_object()
    test_ws_cmd_move_open_loop()
    print("PASS test_teleop_web")
