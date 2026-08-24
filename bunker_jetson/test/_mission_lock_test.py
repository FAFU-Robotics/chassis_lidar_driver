#!/usr/bin/env python3
"""实战锁：任务/开机后忽略 move，不取消自动任务。"""
from __future__ import annotations

import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import bunker_mini.agent as agent_mod
from bunker_mini.agent import AgentState, BunkerMiniAgent


class _FakeWs:
    def send(self, raw: str) -> None:
        pass

    def close(self) -> None:
        pass


class _FakeCtrl:
    def __init__(self) -> None:
        self.stops = 0
        self.velocities: list = []

    def stop_motion(self) -> None:
        self.stops += 1

    def set_velocity(self, v, w) -> None:
        self.velocities.append((v, w))

    def stop(self) -> None:
        pass


def check(name: str, cond: bool, detail: str = "") -> None:
    if cond:
        print(f"  PASS  {name}")
        return
    print(f"  FAIL  {name}  {detail}")
    raise SystemExit(1)


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
            mission_lock=True,
        )
    finally:
        agent_mod.resolve_can_config = orig
    a._controller = _FakeCtrl()
    return a


def test_lock_ignores_move_during_mission() -> None:
    a = _agent()
    stop = threading.Event()

    def _loop() -> None:
        while not stop.is_set():
            time.sleep(0.02)

    t = threading.Thread(target=_loop, daemon=True)
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
            "ts": int(time.time() * 1000),
            "payload": {"action": "move", "v": 0.3, "w": 0.0},
        })
        a._on_message(fake, raw, a._session_id)
        check("实战锁下 move 不得取消任务", t.is_alive() and not a._mission_stop_event.is_set())
        check("实战锁下 move 不得下发速度", a._controller.velocities == [])
        check("mission 仍 recon", a._mission.get("status") == "recon")
    finally:
        stop.set()
        t.join(timeout=1.0)
        a.stop()
        a._cleanup()


def test_lock_allows_estop() -> None:
    a = _agent()
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
            "ts": int(time.time() * 1000),
            "payload": {"action": "estop"},
        })
        a._on_message(fake, raw, a._session_id)
        check("实战锁下 estop 仍停车", a._controller.stops >= 1)
    finally:
        a.stop()
        a._cleanup()


def main() -> None:
    print("mission lock:")
    test_lock_ignores_move_during_mission()
    test_lock_allows_estop()
    print("ALL PASS")


if __name__ == "__main__":
    main()
