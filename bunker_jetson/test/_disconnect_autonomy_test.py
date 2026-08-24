#!/usr/bin/env python3
"""断线不得掐死车侧自主任务；遥控 move 仍必须立即停车。

kb 不灵敏来自云端在控制回路里。find_object / goto / 回放的速度指令在
Jetson 本地闭环。真正危险的是 agent._on_close 旧逻辑：任何 WS 掉线都
stop_motion + 取消 mission —— mock_cloud/SSH 抖动会把溶洞任务掐死。
"""
from __future__ import annotations

import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import bunker_mini.agent as agent_mod
from bunker_mini.agent import AgentState, BunkerMiniAgent, Command


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

    def stop_motion(self) -> None:
        self.stops += 1

    def set_velocity(self, v, w) -> None:
        pass

    def stop(self) -> None:
        pass


class _FakeNav:
    def __init__(self, navigating: bool = False) -> None:
        self.is_navigating = navigating

    def stop(self) -> None:
        self.is_navigating = False


class _FakePlayer:
    def __init__(self, playing: bool = False) -> None:
        self.is_playing = playing

    def stop(self) -> None:
        self.is_playing = False


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
    a._controller = _FakeCtrl()
    return a


def _online(agent: BunkerMiniAgent) -> _FakeWs:
    fake = _FakeWs()
    with agent._lock:
        agent._state = AgentState.ONLINE
        agent._ws = fake
        agent._pending_command = Command(action="move", v=0.2, w=0.0)
        agent._move_deadline = time.time() + 3.0
    return fake


def check(name: str, cond: bool, detail: str = "") -> None:
    if cond:
        print(f"  PASS  {name}")
        return
    print(f"  FAIL  {name}  {detail}")
    raise SystemExit(1)


def test_teleop_stops_on_disconnect() -> None:
    a = _agent()
    try:
        fake = _online(a)
        a._on_close(fake, None, None, a._session_id)
        check("teleop 断线后状态 DISCONNECTED", a.state == AgentState.DISCONNECTED)
        check("teleop 断线立即 stop_motion", a._controller.stops == 1)
        check("teleop 断线清掉 pending move", a._pending_command is None)
    finally:
        a.stop()
        a._cleanup()


def test_mission_survives_disconnect() -> None:
    a = _agent()
    stop = threading.Event()

    def _loop() -> None:
        while not stop.is_set():
            time.sleep(0.02)

    t = threading.Thread(target=_loop, name="fake-mission", daemon=True)
    t.start()
    a._mission_thread = t
    a._mission_stop_event = threading.Event()
    a._mission = {"status": "recon", "target": "岩石A"}
    try:
        fake = _online(a)
        a._on_close(fake, None, None, a._session_id)
        check("任务中断线后状态 DISCONNECTED", a.state == AgentState.DISCONNECTED)
        check("任务中断线不得 stop_motion", a._controller.stops == 0)
        check("任务线程仍活着", t.is_alive())
        check("不得 set mission_stop_event", not a._mission_stop_event.is_set())
        check("mission 状态仍 recon", a._mission.get("status") == "recon")
        check("pending move 已清除（防看门狗误停）", a._pending_command is None)
    finally:
        stop.set()
        t.join(timeout=1.0)
        a.stop()
        a._cleanup()


def test_goto_survives_disconnect() -> None:
    a = _agent()
    a._navigator = _FakeNav(navigating=True)
    try:
        fake = _online(a)
        a._on_close(fake, None, None, a._session_id)
        check("goto 断线不得 stop_motion", a._controller.stops == 0)
        check("goto 仍在导航", a._navigator.is_navigating is True)
    finally:
        a.stop()
        a._cleanup()


def test_replay_survives_disconnect() -> None:
    a = _agent()
    a._player = _FakePlayer(playing=True)
    try:
        fake = _online(a)
        a._on_close(fake, None, None, a._session_id)
        check("回放断线不得 stop_motion", a._controller.stops == 0)
        check("回放仍在进行", a._player.is_playing is True)
    finally:
        a.stop()
        a._cleanup()


def main() -> None:
    print("disconnect / autonomy:")
    test_teleop_stops_on_disconnect()
    test_mission_survives_disconnect()
    test_goto_survives_disconnect()
    test_replay_survives_disconnect()
    print("ALL PASS")


if __name__ == "__main__":
    main()
