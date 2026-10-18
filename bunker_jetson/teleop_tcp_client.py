#!/usr/bin/env python3
"""笔记本 TCP 遥操客户端：HID 双轴 100 Hz 直送工控机 :9100。

必须在【物理键盘所在的电脑】上跑，不要在 SSH 里按 WASD。

    pip install keyboard          # Windows
    python teleop_tcp_client.py --host 192.168.x.x

工控机先开 agent（自带 9100）或: python3 teleop_tcp_server.py --direct-can
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from _bootstrap import ensure_project_root

ensure_project_root()

from _hid_stick import HidStick
from bunker_mini.teleop_tcp import DEFAULT_PORT, DEFAULT_TOKEN, TeleopTcpClient


def main() -> int:
    parser = argparse.ArgumentParser(description="笔记本 → 工控机 TCP 遥操")
    parser.add_argument("--host", default="127.0.0.1", help="工控机 IP（同一 Wi-Fi / 网线）")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--token", default=DEFAULT_TOKEN)
    parser.add_argument("--hz", type=float, default=100.0, help="采样频率（默认 100）")
    parser.add_argument("--v", type=float, default=0.10, help="线速度 m/s")
    parser.add_argument("--w", type=float, default=0.20, help="角速度 rad/s")
    args = parser.parse_args()

    try:
        stick = HidStick.open()
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        if args.host not in ("127.0.0.1", "localhost") and sys.platform.startswith("linux"):
            print(
                "\n你现在是在工控机上跑的。本机遥控请用：\n"
                "  python3 bunker_jetson/teleop_tcp_client.py --host 127.0.0.1\n"
                "笔记本遥控请把本脚本拷到笔记本上执行，--host 才填工控机 IP。",
                file=sys.stderr,
            )
        return 2

    period = 1.0 / max(20.0, min(200.0, args.hz))
    print(f"HID={stick.backend}  →  tcp://{args.host}:{args.port}  {args.hz:.0f} Hz")
    client = TeleopTcpClient(args.host, args.port, args.token)
    try:
        rtt = client.hello()
        print(f"已连接  hello RTT={rtt / 1000.0:.1f} ms")
        print("WASD 驾驶（可同时按） SPACE 急停  +/- 调速  Q 退出")
        v_step, w_step = args.v, args.w
        last_report = 0.0
        rtt_ms = rtt / 1000.0
        plus_prev = minus_prev = space_sent = False
        while True:
            t0 = time.monotonic()
            frame = stick.frame()
            if frame["q"]:
                client.estop()
                print("Q：急停并退出")
                break
            if frame["space"]:
                if not space_sent:
                    client.estop()
                    space_sent = True
            else:
                space_sent = False
                if frame["plus"] and not plus_prev:
                    v_step = min(0.50, v_step + 0.01)
                    w_step = min(1.00, w_step + 0.02)
                if frame["minus"] and not minus_prev:
                    v_step = max(0.05, v_step - 0.01)
                    w_step = max(0.10, w_step - 0.02)
                sv, sw = frame["v"] * v_step, frame["w"] * w_step
                rtt_us = client.stick(sv, sw)
                if rtt_us:
                    rtt_ms = rtt_us / 1000.0
            plus_prev, minus_prev = frame["plus"], frame["minus"]
            now = time.monotonic()
            if now - last_report > 0.5:
                last_report = now
                sys.stdout.write(
                    f"\r  v={frame['v']:+d} w={frame['w']:+d}  "
                    f"cmd={v_step:.2f}/{w_step:.2f}  rtt≈{rtt_ms:.1f}ms   "
                )
                sys.stdout.flush()
            sleep = period - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        try:
            client.estop()
        except Exception:
            pass
        print("\n已中断")
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
