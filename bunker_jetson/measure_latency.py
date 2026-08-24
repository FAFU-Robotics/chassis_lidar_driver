#!/usr/bin/env python3
"""测当前各通道延迟（尽量不让车动）。

测的是「电线上的往返 / 周期」，不是状态栏 1.5 s 刷新。
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from _bootstrap import ensure_project_root

ensure_project_root()

from bunker_mini.teleop_tcp import (
    DEFAULT_PORT,
    DEFAULT_TOKEN,
    TeleopTcpClient,
    TeleopTcpServer,
)


def _pct(xs: list[float], p: float) -> float:
    if not xs:
        return float("nan")
    s = sorted(xs)
    i = min(len(s) - 1, max(0, int(round((p / 100.0) * (len(s) - 1)))))
    return s[i]


def _report(name: str, samples_ms: list[float]) -> None:
    if not samples_ms:
        print(f"  {name}: 无样本")
        return
    print(
        f"  {name}: n={len(samples_ms)}  min={min(samples_ms):.2f}  "
        f"p50={_pct(samples_ms, 50):.2f}  p95={_pct(samples_ms, 95):.2f}  "
        f"max={max(samples_ms):.2f}  mean={statistics.fmean(samples_ms):.2f}  ms"
    )


def bench_tcp_loopback(n: int = 80) -> list[float]:
    applied = {"n": 0}

    def on_stick(_v, _w):
        applied["n"] += 1

    srv = TeleopTcpServer(host="127.0.0.1", port=19100, token=DEFAULT_TOKEN, on_stick=on_stick)
    srv.start()
    time.sleep(0.05)
    samples: list[float] = []
    try:
        cli = TeleopTcpClient("127.0.0.1", 19100, DEFAULT_TOKEN)
        cli.hello()
        for _ in range(n):
            t0 = time.monotonic()
            cli.ping()
            samples.append((time.monotonic() - t0) * 1000.0)
        cli.close()
    finally:
        srv.stop()
    return samples


def bench_tcp_remote(host: str, port: int, token: str, n: int = 60) -> list[float]:
    cli = TeleopTcpClient(host, port, token)
    cli.hello()
    samples = []
    for _ in range(n):
        t0 = time.monotonic()
        cli.ping()
        samples.append((time.monotonic() - t0) * 1000.0)
        time.sleep(0.01)
    cli.close()
    return samples


def bench_ws_query(url: str, device_id: str, bind_code: str, n: int = 20) -> list[float]:
    try:
        import websocket  # type: ignore
    except ImportError:
        print("  WebSocket: 未安装 websocket-client，跳过")
        return []
    samples: list[float] = []
    try:
        ws = websocket.create_connection(url, timeout=3)
        ws.send(json.dumps({
            "type": "auth",
            "deviceId": device_id,
            "ts": int(time.time() * 1000),
            "token": "",
            "payload": {"bindCode": bind_code},
        }))
        token = ""
        deadline = time.time() + 5
        while time.time() < deadline:
            msg = json.loads(ws.recv())
            if msg.get("type") == "auth":
                token = msg.get("token") or ""
                if msg.get("code") == 401 or not token:
                    print("  WebSocket: 鉴权失败")
                    ws.close()
                    return []
                break
        for _ in range(n):
            t0 = time.monotonic()
            ws.send(json.dumps({
                "type": "cmd",
                "deviceId": device_id,
                "ts": int(time.time() * 1000),
                "token": token,
                "payload": {"action": "query"},
            }))
            while time.monotonic() - t0 < 1.5:
                try:
                    ws.settimeout(0.4)
                    msg = json.loads(ws.recv())
                except Exception:
                    break
                if msg.get("type") == "state":
                    samples.append((time.monotonic() - t0) * 1000.0)
                    break
            time.sleep(0.05)
        ws.close()
    except Exception as exc:
        print(f"  WebSocket: 连不上 {url}（{exc}）")
    return samples


def bench_can(seconds: float = 1.2) -> tuple[list[float], list[float]]:
    """0x221 / 0x111 周期。不发运动指令。"""
    fb: list[float] = []
    cmd: list[float] = []
    try:
        import can  # type: ignore
        from bunker_mini.can_util import resolve_can_config
        from bunker_mini.protocol import CanId
    except Exception as exc:
        print(f"  CAN: 跳过（{exc}）")
        return fb, cmd
    try:
        ch, iface = resolve_can_config(None, None, allow_auto_channel=True)
        bus = can.Bus(channel=ch, interface=iface)
    except Exception as exc:
        print(f"  CAN: 打不开（{exc}）")
        return fb, cmd
    last_fb = last_cmd = 0.0
    t_end = time.monotonic() + seconds
    try:
        while time.monotonic() < t_end:
            msg = bus.recv(timeout=0.05)
            if msg is None:
                continue
            now = time.monotonic()
            if msg.arbitration_id == int(CanId.MOTION_FEEDBACK):
                if last_fb:
                    fb.append((now - last_fb) * 1000.0)
                last_fb = now
            elif msg.arbitration_id == int(CanId.MOTION_CONTROL):
                if last_cmd:
                    cmd.append((now - last_cmd) * 1000.0)
                last_cmd = now
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass
    return fb, cmd


def print_budget() -> None:
    print("\n软件路径预算（不是这次测到的，是代码节拍）:")
    print("  旧云端 kb（SSH 字符 + WebSocket JSON）:")
    print("    终端轮询 0-20ms + kb 拍 20ms + 长按刷新 100ms(10Hz)")
    print("    + WS 组包/鉴权/TTL + agent 处理 + 等下一拍 CAN 0-20ms")
    print("    状态栏还要最多 1500ms 才刷新（看屏幕会误当成操作延迟）")
    print("  新 TCP HID（笔记本本机 teleop_from_laptop.py，不是 Cursor 里 kb）:")
    print("    HID 采样 10ms(100Hz) + TCP 帧直达（stick 不 ACK）+ set_velocity_now")
    print("    车侧 0x111 按 10ms/100Hz 下发；电机环改不了，0x221 反馈仍可能约 20ms")
    print("    Cursor/SSH 里 kb 仍是字符流 10～30Hz，测这条路径说明不了老师指标")


def main() -> int:
    parser = argparse.ArgumentParser(description="测 kb / 云端 / TCP / CAN 延迟")
    parser.add_argument("--ws-url", default="ws://127.0.0.1:9000")
    parser.add_argument("--device-id", default="BUNKER-TEST01")
    parser.add_argument("--bind-code", default="TEST-BIND-xxxx")
    parser.add_argument("--tcp-host", default="")
    parser.add_argument("--tcp-port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--token", default=DEFAULT_TOKEN)
    parser.add_argument("--no-can", action="store_true")
    args = parser.parse_args()

    print("=== 延迟实测（不下发非零速度）===\n")

    print("1) TCP 本机回环 ping（协议本身）")
    try:
        _report("TCP loopback ping", bench_tcp_loopback())
    except OSError as exc:
        print(f"  TCP loopback: 失败 {exc}")

    print("\n2) TCP 远程 ping（工控机 :9100 若已在听）")
    host = args.tcp_host or "127.0.0.1"
    try:
        _report(f"TCP {host}:{args.tcp_port} ping", bench_tcp_remote(host, args.tcp_port, args.token))
    except Exception as exc:
        print(f"  TCP {host}:{args.tcp_port}: 未在听（{exc}）")

    print("\n3) 云端 WebSocket query（mock_cloud → agent → state 回来）")
    _report("WS query", bench_ws_query(args.ws_url, args.device_id, args.bind_code))

    if not args.no_can:
        print("\n4) CAN 周期（只听，不发运动）")
        fb, cmd = bench_can()
        _report("0x221 速度反馈周期", fb)
        _report("0x111 指令周期（有人在发才有）", cmd)

    print_budget()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
