"""机械臂网页代理：键盘模式互斥、仿真运动进程、真机命令行。"""

from __future__ import annotations

import fcntl
import json
import socket
import sys
import tempfile
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
if str(_ROOT / "bunker_jetson") not in sys.path:
    sys.path.insert(0, str(_ROOT / "bunker_jetson"))

from arm_proxy import (  # noqa: E402
    MOTION_LOCK,
    OCCUPIED_MSG,
    ArmProxy,
    _MotionLine,
    clamp_cart,
    list_arm_serial_ports,
    lock_held,
)
from bunker_mini.teleop_tcp import TeleopTcpServer  # noqa: E402
from teleop_web import TeleopWebServer, _ws_accept, arm_static_path  # noqa: E402

WEB_PASSWORD = "fafu123456"


class _FakeArm:
    def __init__(self) -> None:
        self.calls: list[tuple] = []
        self.clients: set[str] = set()
        self.stopped = False

    def start(self) -> None:
        return None

    def stop(self) -> None:
        self.stopped = True

    def status(self) -> dict:
        return {
            "enabled": True,
            "kind": "sim",
            "ready": True,
            "occupied": False,
            "allow_motion": True,
            "error": "",
            "port": 1,
        }

    def add_client(self, cid: str) -> None:
        self.clients.add(cid)

    def drop_client(self, cid: str) -> None:
        self.calls.append(("drop", cid))
        self.clients.discard(cid)

    def estop(self, reason: str = "operator") -> None:
        self.calls.append(("e", reason))

    def call(self, cid: str, msg: dict) -> dict:
        self.calls.append((cid, dict(msg)))
        return {"t": "ack", "ok": True, "error": None, "op": msg.get("t")}

    def latest(self):
        return None


def _http(host: str, port: int, raw: bytes) -> bytes:
    sock = socket.create_connection((host, port), timeout=2.0)
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


def _login(host: str, port: int) -> str:
    body = f"password={WEB_PASSWORD}".encode("ascii")
    resp = _http(host, port, (
        b"POST /login HTTP/1.1\r\n"
        + f"Host: {host}\r\n".encode("ascii")
        + b"Content-Type: application/x-www-form-urlencoded\r\n"
        + f"Content-Length: {len(body)}\r\n".encode("ascii")
        + b"Connection: close\r\n\r\n"
        + body
    ))
    head = resp.split(b"\r\n\r\n", 1)[0].decode("iso-8859-1", "replace")
    for line in head.split("\r\n"):
        if line.lower().startswith("set-cookie:") and "bunker_sid=" in line:
            part = line.split(":", 1)[1].strip().split(";", 1)[0]
            return part.split("=", 1)[1].strip()
    raise AssertionError(f"登录失败: {resp[:200]!r}")


def _ws_client(host: str, port: int, sid: str) -> socket.socket:
    sock = socket.create_connection((host, port), timeout=2.0)
    key = "dGhlIHNhbXBsZSBub25jZQ=="
    sock.sendall(
        (
            "GET /ws HTTP/1.1\r\n"
            f"Host: {host}:{port}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Cookie: bunker_sid={sid}\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n"
            "\r\n"
        ).encode("ascii")
    )
    buf = b""
    while b"\r\n\r\n" not in buf:
        chunk = sock.recv(512)
        assert chunk
        buf += chunk
    head = buf.split(b"\r\n\r\n", 1)[0].decode("iso-8859-1")
    assert "101" in head.split("\r\n", 1)[0]
    acc = ""
    for line in head.split("\r\n"):
        if line.lower().startswith("sec-websocket-accept:"):
            acc = line.split(":", 1)[1].strip()
    assert acc == _ws_accept(key)
    return sock


def _ws_send(sock: socket.socket, text: str) -> None:
    payload = text.encode("utf-8")
    mask = b"\x01\x02\x03\x04"
    header = bytes([0x81, 0x80 | len(payload)]) + mask
    masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    sock.sendall(header + masked)


def _ws_texts(sock: socket.socket, timeout: float = 1.0) -> list[str]:
    sock.settimeout(0.2)
    buf = b""
    texts: list[str] = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            chunk = sock.recv(8192)
        except socket.timeout:
            if texts:
                break
            continue
        if not chunk:
            break
        buf += chunk
        while len(buf) >= 2:
            opcode = buf[0] & 0x0F
            masked = buf[1] & 0x80
            n = buf[1] & 0x7F
            off = 2
            if n == 126:
                if len(buf) < 4:
                    break
                n = int.from_bytes(buf[2:4], "big")
                off = 4
            elif n == 127:
                if len(buf) < 10:
                    break
                n = int.from_bytes(buf[2:10], "big")
                off = 10
            mask = b""
            if masked:
                if len(buf) < off + 4:
                    break
                mask = buf[off:off + 4]
                off += 4
            if len(buf) < off + n:
                break
            payload = buf[off:off + n]
            buf = buf[off + n:]
            if masked:
                payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
            if opcode == 1:
                texts.append(payload.decode("utf-8"))
            elif opcode == 8:
                return texts
    return texts


def test_clamp_cart_limits_one_tick():
    out = clamp_cart({"t": "cart", "dxyz": [1, -1, "nope"], "drpy": [90, 0, -9]})
    assert out["dxyz"] == [0.02, -0.02, 0.0]
    assert out["drpy"] == [5.0, 0.0, -5.0]


def test_fafu_command_uses_sdk_python_and_live_flag():
    proxy = ArmProxy(
        enabled=True,
        kind="fafu",
        allow_motion=True,
        python=Path("/tmp/panthera-python"),
        sdk=Path("/home/fafu_robot/zx/fafu_arm_sdk-main"),
        station=Path("/tmp/robot_station"),
    )
    cmd = proxy.command()
    assert cmd[0].endswith("python")
    assert "--motion-only" in cmd
    assert cmd[cmd.index("--arm") + 1] == "fafu"
    assert "--allow-motion" in cmd
    env = proxy.child_env()
    assert env["FAFU_ARM_SDK"].endswith("fafu_arm_sdk-main")
    assert env["STATION_ALLOW_LIVE_ARM"] == "1"
    assert "robot_station" not in cmd[0]


def test_sim_command_does_not_set_live_serial():
    proxy = ArmProxy(
        enabled=True,
        kind="sim",
        python=Path("/tmp/panthera-python"),
        station=Path("/tmp/robot_station"),
    )
    env = proxy.child_env()
    assert "STATION_ALLOW_LIVE_ARM" not in env
    assert "--arm" in proxy.command()
    assert proxy.command()[proxy.command().index("--arm") + 1] == "sim"


def test_lock_held_does_not_spawn():
    lock = Path(tempfile.mkdtemp()) / "motion.lock"
    lock.write_text("1", encoding="utf-8")
    fh = open(lock, "a+", encoding="utf-8")
    fcntl.flock(fh.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    try:
        proxy = ArmProxy(
            enabled=True,
            kind="sim",
            lock_path=lock,
            motion_port=19471,
            python=Path("/home/fafu_robot/miniconda3/envs/panthera/bin/python"),
        )
        proxy.start()
        assert proxy.occupied
        assert proxy.error == OCCUPIED_MSG
        assert proxy._proc is None
    finally:
        fcntl.flock(fh.fileno(), fcntl.LOCK_UN)
        fh.close()
        proxy.stop()


def test_web_arm_mode_zeros_chassis_and_forwards_cart():
    sticks: list[tuple[float, float]] = []
    estops: list[str] = []
    tcp = TeleopTcpServer(
        host="127.0.0.1",
        port=19131,
        token="web-test",
        on_stick=lambda v, w: sticks.append((v, w)),
        on_estop=lambda: estops.append("e"),
    )
    tcp.start()
    time.sleep(0.05)
    arm = _FakeArm()
    web = TeleopWebServer(
        tcp_host="127.0.0.1",
        tcp_port=19131,
        token="web-test",
        http_host="127.0.0.1",
        http_port=19132,
        arm=arm,
    )
    try:
        web.start()
        time.sleep(0.05)
        sid = _login("127.0.0.1", 19132)
        page = _http("127.0.0.1", 19132, (
            b"GET / HTTP/1.1\r\nHost: 127.0.0.1\r\n"
            + f"Cookie: bunker_sid={sid}\r\n".encode("ascii")
            + b"Connection: close\r\n\r\n"
        ))
        assert "机械臂".encode("utf-8") in page
        assert b"btnArmHome" in page
        assert b"btnModeArm" in page
        ws = _ws_client("127.0.0.1", 19132, sid)
        _ws_texts(ws, 0.4)
        _ws_send(ws, json.dumps({"t": "kmode", "mode": "arm"}))
        _ws_send(ws, json.dumps({"t": "s", "v": 0.3, "w": 0.4}))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not sticks:
            time.sleep(0.02)
        assert sticks, "机械臂模式下仍应把底盘打成零速"
        assert all(abs(v) < 1e-6 and abs(w) < 1e-6 for v, w in sticks)
        _ws_send(ws, json.dumps({"t": "cart", "dxyz": [9, 0, 0], "drpy": [0, 0, 0]}))
        _ws_send(ws, json.dumps({"t": "home"}))
        _ws_send(ws, json.dumps({"t": "e"}))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not any(c[0] == "e" for c in arm.calls):
            time.sleep(0.02)
        ops = [c[1].get("t") for c in arm.calls if isinstance(c[0], str) and len(c) == 2 and isinstance(c[1], dict)]
        assert "cart" in ops
        cart = next(c[1] for c in arm.calls if isinstance(c[1], dict) and c[1].get("t") == "cart")
        assert cart["dxyz"][0] == 0.02
        assert "home" in ops
        assert estops, "急停应同时停底盘"
        assert any(c[0] == "e" for c in arm.calls)
        ws.close()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not any(c[0] == "drop" for c in arm.calls):
            time.sleep(0.02)
        assert any(c[0] == "drop" for c in arm.calls), arm.calls
    finally:
        web.stop()
        tcp.stop()
        assert arm.stopped


def _wait_snap(latest, pred, timeout: float = 2.0):
    last = None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        last = latest() or {}
        if pred(last):
            return last
        time.sleep(0.05)
    return last or {}


def _exercise_sim(call, latest, add, drop, estop) -> None:
    """Keyboard cart, home, estop, then clear and drop so a shared sim is not left latched."""
    cid = "web-sim-test"
    add(cid)
    lease = call(cid, {"t": "lease", "action": "take", "name": "web-sim-test"})
    if not lease.get("ok"):
        lease = call(cid, {"t": "lease", "action": "force", "name": "web-sim-test"})
    assert lease.get("ok"), lease
    before = ((latest() or {}).get("arm") or {}).get("q_deg") or [0.0] * 6
    cart = call(cid, {"t": "cart", "dxyz": [0.005, 0, 0], "drpy": [0, 0, 0]})
    assert cart.get("op") == "cart", cart
    if cart.get("ok"):
        def _moved(s: dict) -> bool:
            arm = s.get("arm") or {}
            if arm.get("ik_err"):
                return True
            q = arm.get("q_deg") or []
            return any(
                i < len(q) and abs(float(q[i]) - float(before[i])) > 0.05
                for i in range(min(6, len(before)))
            )

        snap = _wait_snap(latest, _moved)
        arm = snap.get("arm") or {}
        q = arm.get("q_deg") or []
        moved = any(
            abs(float(q[i]) - float(before[i])) > 0.05
            for i in range(min(6, len(q), len(before)))
        )
        assert moved or arm.get("ik_err"), cart
    else:
        assert cart.get("error"), cart
    home = call(cid, {"t": "home"})
    assert home.get("ok"), home
    estop()
    snap = _wait_snap(latest, lambda s: s.get("safety") == "ESTOP_LATCHED")
    assert snap.get("safety") == "ESTOP_LATCHED", snap
    cleared = call(cid, {"t": "clear"})
    assert cleared.get("ok"), cleared
    drop(cid)
    snap = _wait_snap(latest, lambda s: not (s.get("lease") or {}).get("held"))
    assert not (snap.get("lease") or {}).get("held"), snap


def test_sim_keyboard_home_estop_and_disconnect_hold():
    if lock_held(MOTION_LOCK):
        line = _MotionLine("127.0.0.1", 9470)
        line.connect(timeout_s=3.0)
        try:
            _exercise_sim(
                lambda cid, msg: line.call({**msg, "cid": cid}, timeout_s=25.0),
                line.latest,
                lambda cid: line.send({"t": "add", "cid": cid}),
                lambda cid: line.send({"t": "drop", "cid": cid}),
                lambda: line.send({"t": "e", "reason": "test"}),
            )
        finally:
            line.close()
        return
    tmp_path = Path(tempfile.mkdtemp())
    cfg = tmp_path / "station.yaml"
    cfg.write_text(
        "motion_port: 19470\narm: sim\narm_allow_motion: true\nhttp_port: 19400\n",
        encoding="utf-8",
    )
    proxy = ArmProxy(
        enabled=True,
        kind="sim",
        allow_motion=True,
        motion_port=19470,
        lock_path=MOTION_LOCK,
        log_path=tmp_path / "motion.log",
        config_path=cfg,
    )
    proxy.start()
    try:
        assert proxy.ready, proxy.error
        _exercise_sim(
            proxy.call,
            proxy.latest,
            proxy.add_client,
            proxy.drop_client,
            lambda: proxy.estop("test"),
        )
    finally:
        proxy.stop()


def test_arm_static_allowlist_and_page():
    stl, ctype = arm_static_path("/arm-static/meshes/base_link.STL")
    assert stl is not None and stl.name == "base_link.STL"
    assert ctype == "model/stl"
    assert arm_static_path("/arm-static/meshes/link9.STL") is None
    assert arm_static_path("/arm-static/meshes/../base_link.STL") is None
    view, _ = arm_static_path("/arm-static/arm_mesh_view.js")
    three, _ = arm_static_path("/arm-static/three.min.js")
    assert view.is_file() and three.is_file()

    sticks: list[tuple[float, float]] = []
    tcp = TeleopTcpServer(
        host="127.0.0.1", port=19141, token="web-test",
        on_stick=lambda v, w: sticks.append((v, w)),
    )
    tcp.start()
    time.sleep(0.05)
    web = TeleopWebServer(
        tcp_host="127.0.0.1", tcp_port=19141, token="web-test",
        http_host="127.0.0.1", http_port=19142,
        arm=_FakeArm(),
    )
    try:
        web.start()
        time.sleep(0.05)
        blocked = _http("127.0.0.1", 19142, (
            b"GET /arm-static/three.min.js HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n"
        ))
        assert b"401" in blocked.split(b"\r\n", 1)[0]
        sid = _login("127.0.0.1", 19142)
        page = _http("127.0.0.1", 19142, (
            b"GET / HTTP/1.1\r\nHost: 127.0.0.1\r\n"
            + f"Cookie: bunker_sid={sid}\r\n".encode("ascii")
            + b"Connection: close\r\n\r\n"
        ))
        assert b"arm3dHost" in page
        assert "URDF".encode("utf-8") in page
        js = _http("127.0.0.1", 19142, (
            b"GET /arm-static/arm_mesh_view.js HTTP/1.1\r\nHost: 127.0.0.1\r\n"
            + f"Cookie: bunker_sid={sid}\r\n".encode("ascii")
            + b"Connection: close\r\n\r\n"
        ))
        assert b"200" in js.split(b"\r\n", 1)[0]
        assert b"parseSTL" in js
        missing = _http("127.0.0.1", 19142, (
            b"GET /arm-static/meshes/link9.STL HTTP/1.1\r\nHost: 127.0.0.1\r\n"
            + f"Cookie: bunker_sid={sid}\r\n".encode("ascii")
            + b"Connection: close\r\n\r\n"
        ))
        assert b"404" in missing.split(b"\r\n", 1)[0]
        mesh = _http("127.0.0.1", 19142, (
            b"GET /arm-static/meshes/link1.STL HTTP/1.1\r\nHost: 127.0.0.1\r\n"
            + f"Cookie: bunker_sid={sid}\r\n".encode("ascii")
            + b"Connection: close\r\n\r\n"
        ))
        head, _, body = mesh.partition(b"\r\n\r\n")
        assert b"200" in head.split(b"\r\n", 1)[0]
        assert b"model/stl" in head
        assert len(body) == stl.with_name("link1.STL").stat().st_size
    finally:
        web.stop()
        tcp.stop()


def test_auto_uses_one_debug_board_and_sim_without_one():
    """一块调试板 → 真机键盘并钉住串口；没有或多块 → 仿真，不开 USB。"""
    ports = list_arm_serial_ports()
    proxy = ArmProxy(
        enabled=True,
        kind="auto",
        python=Path("/tmp/panthera-python"),
        station=Path("/tmp/robot_station"),
    )
    if len(ports) == 1:
        assert proxy.kind == "fafu"
        assert proxy.port == ports[0]
        assert proxy.allow_motion
        cmd = proxy.command()
        assert cmd[cmd.index("--arm") + 1] == "fafu"
        assert "--allow-motion" in cmd
        env = proxy.child_env()
        assert env["STATION_ALLOW_LIVE_ARM"] == "1"
        assert env["STATION_ARM_PORT"] == ports[0]
    else:
        assert proxy.kind == "sim"
        assert proxy.port == ""
        assert "STATION_ALLOW_LIVE_ARM" not in proxy.child_env()
        assert "STATION_ARM_PORT" not in proxy.child_env()


def test_plain_fafu_stays_read_only_until_allowed():
    proxy = ArmProxy(
        enabled=True,
        kind="fafu",
        python=Path("/tmp/panthera-python"),
        station=Path("/tmp/robot_station"),
    )
    assert proxy.allow_motion is False
    assert "--allow-motion" not in proxy.command()


if __name__ == "__main__":
    test_clamp_cart_limits_one_tick()
    test_fafu_command_uses_sdk_python_and_live_flag()
    test_sim_command_does_not_set_live_serial()
    test_lock_held_does_not_spawn()
    test_web_arm_mode_zeros_chassis_and_forwards_cart()
    test_arm_static_allowlist_and_page()
    test_sim_keyboard_home_estop_and_disconnect_hold()
    test_auto_uses_one_debug_board_and_sim_without_one()
    test_plain_fafu_stays_read_only_until_allowed()
    print("PASS test_arm_proxy")
