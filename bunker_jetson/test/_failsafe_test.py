#!/usr/bin/env python3
"""Failsafe: command TTL, link class, power class, checkpoint, stale move gate."""
from __future__ import annotations

import json
import os
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from bunker_mini.failsafe import (
    FailsafeMonitor,
    FailsafePolicy,
    LinkClass,
    PowerClass,
    atomic_write_json,
    classify_link,
    classify_power,
    command_age_s,
    judge_command,
    load_checkpoint,
    should_return_on_power,
)
import bunker_mini.agent as agent_mod
from bunker_mini.agent import AgentState, BunkerMiniAgent, Command


def check(name: str, cond: bool, detail: str = "") -> None:
    if cond:
        print(f"  PASS  {name}")
        return
    print(f"  FAIL  {name}  {detail}")
    raise SystemExit(1)


def test_command_ttl() -> None:
    p = FailsafePolicy()
    now = 1_700_000_000.0
    fresh_ts = int((now - 0.2) * 1000)
    stale_move = int((now - 2.0) * 1000)
    stale_fo = int((now - 8.0) * 1000)

    v = judge_command("move", fresh_ts, LinkClass.HEALTHY, p, now=now)
    check("fresh move accepted", v.accept)

    v = judge_command("move", stale_move, LinkClass.HEALTHY, p, now=now)
    check("stale move rejected", not v.accept and "stale" in v.reason)

    v = judge_command("estop", stale_move, LinkClass.HEALTHY, p, now=now)
    check("stale estop still accepted", v.accept)

    v = judge_command("cancel", stale_fo, LinkClass.LOST, p, now=now)
    check("cancel accepted on lost link", v.accept)

    v = judge_command("find_object", stale_fo, LinkClass.HEALTHY, p, now=now)
    check("stale find_object rejected", not v.accept)

    v = judge_command("find_object", fresh_ts, LinkClass.HEALTHY, p, now=now)
    check("fresh find_object accepted", v.accept)

    v = judge_command("move", None, LinkClass.HEALTHY, p, now=now)
    check("legacy move without ts accepted when healthy", v.accept)

    v = judge_command("move", None, LinkClass.DEGRADED, p, now=now)
    check("legacy move without ts rejected when degraded", not v.accept)

    v = judge_command("move", fresh_ts, LinkClass.LOST, p, now=now)
    check("move rejected on lost link", not v.accept)

    age = command_age_s(int((now + 0.5) * 1000), now=now, clock_skew_s=2.0)
    check("cloud-ahead clock clamped to 0", age == 0.0)


def test_link_and_power() -> None:
    p = FailsafePolicy()
    check("silence 0 healthy", classify_link(rx_age_s=0.2, rtt_s=0.05, policy=p) is LinkClass.HEALTHY)
    check("rtt 0.6 degraded", classify_link(rx_age_s=0.2, rtt_s=0.6, policy=p) is LinkClass.DEGRADED)
    check("silence 4 degraded", classify_link(rx_age_s=4.0, rtt_s=0.05, policy=p) is LinkClass.DEGRADED)
    check("silence 12 lost", classify_link(rx_age_s=12.0, rtt_s=0.05, policy=p) is LinkClass.LOST)

    check("soc 50 ok", classify_power(soc_percent=50.0, policy=p) is PowerClass.OK)
    check("soc 18 warn", classify_power(soc_percent=18.0, policy=p) is PowerClass.WARN)
    check("soc 10 critical", classify_power(soc_percent=10.0, policy=p) is PowerClass.CRITICAL)
    check("no sensor not empty", classify_power(soc_percent=None, voltage_v=0.0, policy=p) is PowerClass.OK)
    check("undervoltage fault critical",
          classify_power(undervoltage_fault=True, soc_percent=80.0, policy=p) is PowerClass.CRITICAL)
    check("return on recon critical", should_return_on_power("recon", PowerClass.CRITICAL))
    check("no return on returning", not should_return_on_power("returning", PowerClass.CRITICAL))
    check("no return on warn", not should_return_on_power("recon", PowerClass.WARN))
    check("no return without mission", not should_return_on_power(None, PowerClass.CRITICAL))


def test_checkpoint_atomic() -> None:
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "failsafe_checkpoint.json"
        atomic_write_json(path, {"schema": 1, "mission": {"status": "recon"}})
        data = load_checkpoint(path)
        check("checkpoint loads", bool(data and data["mission"]["status"] == "recon"))
        mon = FailsafeMonitor(checkpoint_path=path)
        boot = mon.load_boot_checkpoint()
        check("monitor loads boot checkpoint", boot == data)


def _agent() -> BunkerMiniAgent:
    orig = agent_mod.resolve_can_config

    def _fake_resolve(channel, interface, *, allow_auto_channel=True):
        return "0", "virtual"

    agent_mod.resolve_can_config = _fake_resolve
    try:
        a = BunkerMiniAgent(
            ws_url="ws://127.0.0.1:1/test",
            device_id="TEST-01",
            bind_code="TEST-BIND-xxxx",
        )
    finally:
        agent_mod.resolve_can_config = orig
    return a


class _FakeWs:
    def __init__(self) -> None:
        self.sent: list[str] = []

    def send(self, raw: str) -> None:
        self.sent.append(raw)

    def close(self) -> None:
        pass


class _FakeCtrl:
    def __init__(self) -> None:
        self.stops = 0
        self.velocities: list[tuple[float, float]] = []
        self.latest_status = None
        self.latest_bms = None

    def stop_motion(self) -> None:
        self.stops += 1

    def set_velocity(self, v, w) -> None:
        self.velocities.append((v, w))

    def stop(self) -> None:
        pass


def test_stale_move_does_not_kill_mission() -> None:
    a = _agent()
    a._controller = _FakeCtrl()
    stop = threading.Event()

    def _loop() -> None:
        while not stop.is_set():
            time.sleep(0.02)

    t = threading.Thread(target=_loop, name="fake-mission", daemon=True)
    t.start()
    a._mission_thread = t
    a._mission_stop_event = threading.Event()
    a._mission = {"status": "recon", "target": "岩石A"}
    fake = _FakeWs()
    with a._lock:
        a._state = AgentState.ONLINE
        a._ws = fake
        a._token = "tok"
    a._failsafe.reset_session()
    try:
        raw = json.dumps({
            "type": "cmd",
            "deviceId": "TEST-01",
            "token": "tok",
            "ts": int((time.time() - 10.0) * 1000),
            "payload": {"action": "move", "v": 0.3, "w": 0.0},
        })
        a._on_message(fake, raw, a._session_id)
        check("stale move 不得取消任务线程", t.is_alive())
        check("stale move 不得 set stop event", not a._mission_stop_event.is_set())
        check("mission 仍 recon", a._mission.get("status") == "recon")
        check("stale move 未下发速度", a._controller.velocities == [])
    finally:
        stop.set()
        t.join(timeout=1.0)
        a.stop()
        a._cleanup()


def test_fresh_estop_accepted() -> None:
    a = _agent()
    a._controller = _FakeCtrl()
    fake = _FakeWs()
    with a._lock:
        a._state = AgentState.ONLINE
        a._ws = fake
        a._token = "tok"
    a._failsafe.reset_session()
    try:
        raw = json.dumps({
            "type": "cmd",
            "deviceId": "TEST-01",
            "token": "tok",
            "ts": int((time.time() - 30.0) * 1000),
            "payload": {"action": "estop"},
        })
        a._on_message(fake, raw, a._session_id)
        check("过期 estop 仍执行停车", a._controller.stops >= 1)
    finally:
        a.stop()
        a._cleanup()


def test_boot_checkpoint_does_not_drive() -> None:
    a = _agent()
    a._controller = _FakeCtrl()
    try:
        a._failsafe.checkpoint_path = Path(tempfile.mkdtemp()) / "failsafe_checkpoint.json"
        atomic_write_json(a._failsafe.checkpoint_path, {
            "schema": 1,
            "mission": {"status": "navigating", "target": "岩石A"},
            "pose": {"x": 1.2, "y": 0.4, "yawDeg": 30},
        })
        a._restore_failsafe_checkpoint()
        check("boot 标记 interrupted", a._mission.get("status") == "interrupted")
        check("boot 不自动开车", a._controller.velocities == [] and a._controller.stops == 0)
        check("保留 lastStatus", a._mission.get("lastStatus") == "navigating")
    finally:
        a.stop()
        a._cleanup()


def main() -> None:
    print("failsafe:")
    test_command_ttl()
    test_link_and_power()
    test_checkpoint_atomic()
    test_stale_move_does_not_kill_mission()
    test_fresh_estop_accepted()
    test_boot_checkpoint_does_not_drive()
    print("ALL PASS")


if __name__ == "__main__":
    main()
