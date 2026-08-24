"""TCP 遥操协议：双轴独立、本机 RTT、松一轴留另一轴。"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from bunker_mini.teleop_tcp import TeleopTcpClient, TeleopTcpServer  # noqa: E402


def test_tcp_ping_and_stick_axes():
    sticks: list[tuple[float, float]] = []
    srv = TeleopTcpServer(
        host="127.0.0.1",
        port=19101,
        token="test-token",
        on_stick=lambda v, w: sticks.append((v, w)),
    )
    srv.start()
    time.sleep(0.05)
    try:
        cli = TeleopTcpClient("127.0.0.1", 19101, "test-token")
        rtt = cli.hello()
        assert rtt < 50_000, f"本机 hello RTT 应远小于 50ms，实际 {rtt}us"
        ping = cli.ping()
        assert ping < 50_000
        cli.stick(0.10, 0.20, wait_ack=True)
        cli.stick(0.10, 0.00, wait_ack=True)
        time.sleep(0.02)
        assert any(abs(v - 0.10) < 1e-3 and abs(w - 0.20) < 1e-3 for v, w in sticks)
        assert any(abs(v - 0.10) < 1e-3 and abs(w) < 1e-3 for v, w in sticks)
        cli.close()
    finally:
        srv.stop()


def test_tcp_cmd_and_event():
    cmds: list[dict] = []
    srv = TeleopTcpServer(
        host="127.0.0.1",
        port=19103,
        token="test-token",
        on_cmd=lambda p: cmds.append(p),
    )
    srv.start()
    time.sleep(0.05)
    try:
        cli = TeleopTcpClient("127.0.0.1", 19103, "test-token")
        cli.hello()
        cli.cmd({"action": "find_object", "name": "target", "approach": True})
        time.sleep(0.05)
        assert cmds and cmds[0]["action"] == "find_object"
        assert cmds[0]["name"] == "target"
        srv.push_event({"type": "event", "payload": {"event": "info", "msg": "ok"}})
        time.sleep(0.05)
        evs = cli.poll_events()
        assert any(
            (e.get("payload") or {}).get("event") == "info" for e in evs
        ), evs
        cli.close()
    finally:
        srv.stop()


def test_stick_default_no_ack_and_cmd_does_not_block():
    """stick 默认不等 ACK；cmd 等 ACK 时另一线程仍能发 stick。"""
    sticks: list[tuple[float, float]] = []
    started = threading.Event()

    def slow_cmd(_payload: dict) -> None:
        started.set()
        time.sleep(0.25)

    srv = TeleopTcpServer(
        host="127.0.0.1",
        port=19104,
        token="test-token",
        on_stick=lambda v, w: sticks.append((v, w)),
        on_cmd=slow_cmd,
    )
    srv.start()
    time.sleep(0.05)
    try:
        cli = TeleopTcpClient("127.0.0.1", 19104, "test-token")
        cli.hello()
        t0 = time.monotonic()
        cli.stick(0.11, 0.0)
        assert time.monotonic() - t0 < 0.05, "默认 stick 不应等 ACK"
        time.sleep(0.03)
        assert any(abs(v - 0.11) < 1e-3 for v, _w in sticks)

        def _cmd() -> None:
            cli.cmd({"action": "query"})

        th = threading.Thread(target=_cmd, daemon=True)
        th.start()
        assert started.wait(1.0)
        before = len(sticks)
        t1 = time.monotonic()
        cli.stick(0.22, 0.0)
        assert time.monotonic() - t1 < 0.05
        time.sleep(0.03)
        assert any(abs(v - 0.22) < 1e-3 for v, _w in sticks[before:])
        th.join(timeout=1.0)
        cli.close()
    finally:
        srv.stop()


def test_bad_token_rejected():
    srv = TeleopTcpServer(host="127.0.0.1", port=19102, token="secret")
    srv.start()
    time.sleep(0.05)
    try:
        cli = TeleopTcpClient("127.0.0.1", 19102, "wrong")
        try:
            cli.hello()
            raised = False
        except PermissionError:
            raised = True
        cli.close()
        assert raised
    finally:
        srv.stop()


if __name__ == "__main__":
    test_tcp_ping_and_stick_axes()
    test_tcp_cmd_and_event()
    test_stick_default_no_ack_and_cmd_does_not_block()
    test_bad_token_rejected()
    print("PASS test_teleop_tcp")
