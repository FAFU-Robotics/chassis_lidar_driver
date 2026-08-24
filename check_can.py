#!/usr/bin/env python3
"""本地终端检测 CAN：适配器、网卡、是否听到底盘 0x211。不需要 run_local。

    python3 check_can.py
    python3 check_can.py --seconds 3
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
JETSON = ROOT / "bunker_jetson"
if str(JETSON) not in sys.path:
    sys.path.insert(0, str(JETSON))

from bunker_mini.can_util import (  # noqa: E402
    CHASSIS_RX_IDS,
    create_can_bus,
    list_socketcan_interfaces,
    socketcan_ctrl_state,
    socketcan_is_listen_only,
    socketcan_operstate,
    usb_socketcan_channels,
)


def _lsusb_candle() -> str:
    try:
        out = subprocess.check_output(["lsusb"], text=True, timeout=3)
    except Exception as exc:
        return f"  lsusb 失败: {exc}"
    lines = [
        ln for ln in out.splitlines()
        if "1d50:606f" in ln or "1209:2323" in ln or "CAN" in ln
    ]
    return "\n".join(f"  {ln}" for ln in lines) or "  （lsusb 没有 candleLight 1d50:606f / 1209:2323）"


def _kind(name: str) -> str:
    try:
        dev = os.path.realpath(os.path.join("/sys/class/net", name, "device"))
    except OSError:
        return "?"
    if "mttcan" in dev:
        return "板载 mttcan"
    if "usb" in dev.lower():
        return "USB-CAN"
    return "其它"


def _stats(name: str) -> tuple[int, int, int]:
    base = f"/sys/class/net/{name}/statistics"
    def _n(fn: str) -> int:
        try:
            return int(Path(base, fn).read_text().strip())
        except (OSError, ValueError):
            return 0
    return _n("rx_packets"), _n("tx_packets"), _n("tx_dropped")


def _gs_usb_xmit_fail() -> list[str]:
    try:
        out = subprocess.check_output(
            ["journalctl", "-k", "--since", "2 hours ago", "--no-pager"],
            text=True, timeout=5,
        )
    except Exception:
        return []
    lines = [ln for ln in out.splitlines() if "usb xmit fail" in ln]
    return lines[-6:]


def _bitrate(name: str) -> str:
    import re
    try:
        out = subprocess.check_output(
            ["ip", "-details", "link", "show", name], text=True, timeout=2,
        )
    except Exception:
        return "?"
    m = re.search(r"bitrate (\d+)", out)
    return m.group(1) if m else "?"


def _listen(channel: str, seconds: float) -> list[int]:
    ids: list[int] = []
    try:
        bus = create_can_bus(channel, "socketcan")
    except Exception as exc:
        print(f"  {channel}: 打不开（{exc}）")
        return ids
    deadline = time.time() + seconds
    try:
        while time.time() < deadline:
            msg = bus.recv(timeout=0.2)
            if msg is None:
                continue
            ids.append(int(msg.arbitration_id))
            if len(ids) >= 12:
                break
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass
    return ids


def _holders() -> str:
    try:
        text = Path("/proc/net/can/rcvlist_all").read_text()
    except OSError:
        return "  （读不到 /proc/net/can）"
    if "no entry" in text and text.count("no entry") >= 2:
        return "  当前没有进程占用 SocketCAN 接收表"
    return "  " + " ".join(text.split())


def _agent_hint() -> str:
    try:
        raw = Path("/tmp/bunker_chassis_status").read_text(encoding="utf-8")
        data = json.loads(raw)
    except Exception:
        return "  run_local 未在跑，或还没有写出 /tmp/bunker_chassis_status"
    heard = data.get("heard")
    ch = data.get("channel")
    mode = data.get("mode") or "—"
    return f"  上次 agent 快照: channel={ch} heard={heard} mode={mode}"


def report(seconds: float = 2.0) -> int:
    seconds = max(0.5, float(seconds))

    print("=== 1. USB 适配器 ===")
    print(_lsusb_candle())

    print("\n=== 2. SocketCAN 网卡 ===")
    ifaces = list_socketcan_interfaces()
    if not ifaces:
        print("  没有 can* 网卡。先: bash bunker_jetson/bringup_gs_usb_can.sh")
        return 2
    for name in ifaces:
        rx, tx, drop = _stats(name)
        lo = " LISTEN-ONLY" if socketcan_is_listen_only(name) else ""
        extra = f" TXdrop={drop}" if drop else ""
        print(
            f"  {name}: {socketcan_operstate(name):<4}  {_kind(name):<12}  "
            f"{socketcan_ctrl_state(name)}  {_bitrate(name)}bps  "
            f"RX={rx} TX={tx}{extra}{lo}"
        )
    usb = usb_socketcan_channels()
    print(f"  USB-CAN 口: {', '.join(usb) if usb else '（无）'}")

    print("\n=== 3. 谁占用了 CAN ===")
    print(_holders())
    print(_agent_hint())

    print(f"\n=== 4. 听 {seconds:.1f}s（找底盘 0x211 / 0x221 / 0x361）===")
    hits: dict[str, list[int]] = {}
    for name in ifaces:
        ids = _listen(name, seconds)
        hits[name] = ids
        if ids:
            shown = " ".join(f"0x{i:X}" for i in ids[:8])
            print(f"  {name}: {len(ids)} 帧  {shown}")
        else:
            print(f"  {name}: 0 帧")

    print("\n=== 5. 结论 ===")
    chassis = [
        name for name, ids in hits.items()
        if any(i in CHASSIS_RX_IDS for i in ids)
    ]
    if chassis:
        print(f"  底盘在线，口在 {', '.join(chassis)}（听到了 0x211 一类反馈）。")
        print("  再开: python3 run_local.py")
        return 0
    if usb:
        xmit = _gs_usb_xmit_fail()
        rx, tx, drop = _stats(usb[0])
        print(f"  USB-CAN {usb[0]} 在，但 {seconds:.0f}s 内没有底盘帧。")
        if xmit or (tx == 0 and drop > 0):
            print("  ★ 适配器 USB 发送已失败（gs_usb usb xmit fail / TX=0 且有 TXdrop）。")
            print("    帧到不了 CAN 线，也听不到 0x211。不要再软件 usbreset。")
            print("    请物理拔掉 candleLight，等 3 秒再插回，然后:")
            print("      bash bunker_jetson/bringup_gs_usb_can.sh")
            print("      python3 check_can.py")
            if xmit:
                print("    最近内核:")
                for ln in xmit[-3:]:
                    print("     ", ln.split("fafurobot")[-1].strip() if "fafurobot" in ln else ln[-90:])
        else:
            print("  网卡看起来健康。工控机有电 ≠ 底盘 CAN 节点在广播。")
            print("  再查底盘钥匙/急停（和工控机供电不是一路）。")
        print("  持续看帧: candump", usb[0])
    else:
        print("  没有 USB-CAN。插入 candleLight 后:")
        print("    bash bunker_jetson/bringup_gs_usb_can.sh")
    return 1


def main() -> int:
    parser = argparse.ArgumentParser(description="本地检测 CAN / 底盘 0x211")
    parser.add_argument("--seconds", type=float, default=2.0, help="每个口监听秒数")
    args = parser.parse_args()
    return report(args.seconds)


if __name__ == "__main__":
    raise SystemExit(main())
