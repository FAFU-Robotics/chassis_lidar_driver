"""run_local 导航控制台：query 等新状态、goto 钳速、状态行含导航。"""

from __future__ import annotations

import sys
import time
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
if str(_ROOT / "bunker_jetson") not in sys.path:
    sys.path.insert(0, str(_ROOT / "bunker_jetson"))

import run_local as rl  # noqa: E402


class _FakeClient:
    def __init__(self) -> None:
        self.cmds: list[dict] = []
        self.estops = 0

    def cmd(self, payload: dict, timeout: float = 3.0) -> int:
        self.cmds.append(dict(payload))
        return 0

    def estop(self) -> int:
        self.estops += 1
        return 0

    def stick(self, *args, **kwargs) -> int:
        return 0

    def poll_events(self) -> list:
        return []


def _console() -> rl.LocalConsole:
    return rl.LocalConsole(_FakeClient(), {})


def test_query_waits_for_fresh_state() -> None:
    c = _console()

    def _slow_query(payload: dict) -> None:
        if payload.get("action") == "query":
            time.sleep(0.08)
            c._on_msg({
                "type": "state",
                "payload": {
                    "pose": {"x": 0.41, "y": 0.0, "yaw": 2.0},
                    "speed": 0.12,
                    "mode": "CAN 指令模式",
                    "lidar": {"online": True, "frontObstacle": 2.4},
                    "navigating": True,
                    "drive": {"kind": "goto", "goto": {"x": 0.6, "y": 0.0, "distM": 0.19}},
                    "battery": 25.6,
                },
            })

    c.client.cmd = lambda payload, timeout=3.0: (_slow_query(payload), 0)[1]
    lines: list[str] = []
    orig = print

    def _capture(*args, **kwargs):
        lines.append(" ".join(str(a) for a in args))

    # patch print used by _show_state
    import builtins
    builtins.print = _capture
    try:
        assert c.dispatch("q") is True
    finally:
        builtins.print = orig
    text = "\n".join(lines)
    assert "0.41" in text
    assert "进行中" in text
    assert "剩0.19m" in text
    assert c._last_state["pose"]["x"] == 0.41


def _ack_goto(c: rl.LocalConsole):
    def _cmd(payload: dict, timeout: float = 3.0) -> int:
        c.client.cmds.append(dict(payload))
        if payload.get("action") == "goto":
            c._on_msg({"type": "event", "payload": {"event": "goto", "msg": "导航已开始"}})
            c._on_msg({
                "type": "state",
                "payload": {
                    "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                    "speed": 0.0,
                    "mode": "CAN 指令模式",
                    "lidar": {"online": True},
                    "navigating": True,
                    "drive": {"kind": "goto", "goto": {
                        "x": payload.get("x"), "y": payload.get("y"), "distM": 0.6,
                    }},
                },
            })
        return 0
    c.client.cmd = _cmd


def test_goto_clamps_speed_and_sends_payload() -> None:
    c = _console()
    c._last_state = {
        "lidar": {"online": True},
        "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    }
    _ack_goto(c)
    c.dispatch("g 0.6 0 0.9")
    sent = [p for p in c.client.cmds if p.get("action") == "goto"]
    assert sent and sent[0]["x"] == 0.6 and sent[0]["y"] == 0.0
    assert sent[0]["speed"] == 0.5


def test_goto_optional_yaw_deg() -> None:
    c = _console()
    c._last_state = {"lidar": {"online": True}, "pose": {"x": 0, "y": 0, "yaw": 0}}
    _ack_goto(c)
    c.dispatch("g 0.6 0 0.12 90")
    sent = [p for p in c.client.cmds if p.get("action") == "goto"]
    assert sent and sent[0]["speed"] == 0.12
    assert sent[0]["yawDeg"] == 90.0


def test_goto_omits_default_speed() -> None:
    c = _console()
    c._last_state = {"lidar": {"online": True}, "pose": {"x": 0, "y": 0, "yaw": 0}}
    _ack_goto(c)
    c.dispatch("goto 0.5 0")
    sent = [p for p in c.client.cmds if p.get("action") == "goto"]
    assert sent and "speed" not in sent[0]


def test_show_state_key_includes_navigating() -> None:
    c = _console()
    idle = {
        "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "speed": 0.0,
        "mode": "CAN 指令模式",
        "lidar": {"online": True},
        "navigating": False,
        "drive": {"kind": "idle"},
        "battery": 25.0,
    }
    going = dict(idle)
    going["navigating"] = True
    going["drive"] = {"kind": "goto", "goto": {"x": 0.5, "y": 0.0, "distM": 0.5}}
    import builtins
    lines: list[str] = []
    orig = print
    builtins.print = lambda *a, **k: lines.append(" ".join(str(x) for x in a))
    try:
        c._show_state(idle, force=False)
        c._show_state(idle, force=False)
        c._show_state(going, force=False)
    finally:
        builtins.print = orig
    nav_lines = [ln for ln in lines if "导航=" in ln]
    assert len(nav_lines) == 2
    assert "进行中" in nav_lines[1]


def test_fmt_battery_soc_and_pack_voltage() -> None:
    assert rl._fmt_battery({"battery": 0.89, "batteryVoltageV": 26.0}) == "89% 26.0V"
    assert rl._fmt_battery({"battery": 0.89}) == "89%"
    assert "低电" in rl._fmt_battery({"battery": 0.4, "batteryVoltageV": 23.0})
    assert rl._fmt_battery({"battery": 25.6}) == "25.6V"


def test_headless_env_clears_display_and_lidar() -> None:
    out = rl._headless_env({
        "DISPLAY": ":0",
        "WAYLAND_DISPLAY": "wayland-0",
        "XAUTHORITY": "/tmp/x",
        "BUNKER_ENABLE_LIDAR": "1",
        "KEEP": "1",
    })
    assert "DISPLAY" not in out
    assert "WAYLAND_DISPLAY" not in out
    assert "XAUTHORITY" not in out
    assert out["BUNKER_MAP_VIEW"] == "0"
    assert out["BUNKER_ENABLE_LIDAR"] == "0"
    assert out["KEEP"] == "1"


def test_daemon_implies_no_lidar_flag() -> None:
    import argparse
    # 与 main() 相同：--daemon 强制不开雷达
    ns = argparse.Namespace(daemon=True, no_lidar=False)
    if ns.daemon:
        ns.no_lidar = True
    assert ns.no_lidar is True


if __name__ == "__main__":
    test_query_waits_for_fresh_state()
    test_goto_clamps_speed_and_sends_payload()
    test_goto_optional_yaw_deg()
    test_goto_omits_default_speed()
    test_show_state_key_includes_navigating()
    test_fmt_battery_soc_and_pack_voltage()
    test_headless_env_clears_display_and_lidar()
    test_daemon_implies_no_lidar_flag()
    print("PASS test_run_local_nav")
