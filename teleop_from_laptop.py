#!/usr/bin/env python3
"""笔记本本机遥控：读这台电脑的键盘 HID，100Hz TCP 送到工控机 :9100。

这才是老师要的控制通道（不要用 SSH / Cursor 底部终端当遥控总线）：

    笔记本 HID 100Hz  →  局域网 TCP :9100  →  set_velocity_now  →  CAN 100Hz

Cursor 打开的是工控机硬盘。远程终端里按的是字符流，没有 key-up，
多键/松键只能猜，那条路到不了 100Hz、也到不了 ~20ms。

用法（笔记本本机 PowerShell / CMD，不要在 Cursor 远程窗里）：

    python teleop_from_laptop.py --host 192.168.x.x

Windows 用系统 API 读按键，一般不用装包。
工控机上先开着 python3 run_local.py（已经在听 9100）。
"""

from __future__ import annotations

import argparse
import os
import socket
import struct
import sys
import threading
import time
from pathlib import Path

MAGIC = 0xB1CA
HEADER = struct.Struct("<HBBHIH")
STICK = struct.Struct("<hhB")
TYPE_HELLO, TYPE_STICK, TYPE_ESTOP, TYPE_ACK = 1, 3, 5, 0x81
DEFAULT_PORT = 9100
DEFAULT_TOKEN = "bunker-teleop"


def _on_jetson() -> bool:
    return Path("/etc/nv_tegra_release").is_file()


def _ssh() -> bool:
    return bool(os.environ.get("SSH_CONNECTION") or os.environ.get("SSH_CLIENT"))


def _lan_ips() -> list[str]:
    ips: list[str] = []
    try:
        out = os.popen("hostname -I 2>/dev/null").read().split()
        for tok in out:
            if tok.count(".") == 3 and not tok.startswith("127."):
                ips.append(tok)
    except Exception:
        pass
    return ips


def now_us() -> int:
    return int(time.monotonic() * 1_000_000) & 0xFFFFFFFF


def pack(typ: int, payload: bytes = b"", seq: int = 0) -> bytes:
    return HEADER.pack(MAGIC, typ, 0, seq & 0xFFFF, now_us(), len(payload)) + payload


class Stick:
    def __init__(self, down, backend: str) -> None:
        self._down = down
        self.backend = backend

    def frame(self) -> dict:
        v = (1 if self._down("w") else 0) - (1 if self._down("s") else 0)
        w = (1 if self._down("a") else 0) - (1 if self._down("d") else 0)
        return {
            "v": v, "w": w,
            "q": self._down("q"),
            "space": self._down("space"),
            "plus": self._down("+") or self._down("="),
            "minus": self._down("-"),
        }


def open_stick() -> Stick:
    if sys.platform.startswith("win"):
        user32 = __import__("ctypes").windll.user32
        vk = {
            "w": 0x57, "a": 0x41, "s": 0x53, "d": 0x44, "q": 0x51,
            "space": 0x20, "+": 0xBB, "=": 0xBB, "-": 0xBD,
        }
        extra = { "+": (0x6B,), "-": (0x6D,) }

        def down(name: str) -> bool:
            codes = (vk[name],) + extra.get(name, ())
            return any(user32.GetAsyncKeyState(c) & 0x8000 for c in codes)

        return Stick(down, "win32-GetAsyncKeyState")

    try:
        import keyboard  # type: ignore

        keyboard.is_pressed("w")
        return Stick(lambda n: bool(keyboard.is_pressed(n)), "keyboard")
    except Exception as exc:
        raise RuntimeError(
            "读不到本机键盘。Windows 应直接能用；其它系统: pip install keyboard\n"
            f"详情: {exc}"
        ) from exc


class Client:
    def __init__(self, host: str, port: int, token: str) -> None:
        self.sock = socket.create_connection((host, port), timeout=5.0)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._seq = 0
        self._send_lock = threading.Lock()
        self._rx_stop = threading.Event()
        self._rx: threading.Thread | None = None

    def close(self) -> None:
        self._rx_stop.set()
        try:
            self.sock.close()
        except OSError:
            pass
        if self._rx is not None:
            self._rx.join(timeout=1.0)

    def _send(self, typ: int, payload: bytes = b"") -> None:
        self._seq = (self._seq + 1) & 0xFFFF
        with self._send_lock:
            self.sock.sendall(pack(typ, payload, self._seq))

    def _rx_loop(self) -> None:
        """后台丢掉 ACK/状态推送。绝不能放在 100Hz stick 热路径里 recv。"""
        self.sock.settimeout(0.2)
        while not self._rx_stop.is_set():
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break

    def hello(self, token: str) -> None:
        self._send(TYPE_HELLO, token.encode("utf-8"))
        self.sock.settimeout(2.0)
        hdr = self.sock.recv(HEADER.size)
        if len(hdr) < HEADER.size:
            raise OSError("工控机无应答，确认 run_local.py 已开且 9100 能通")
        _magic, typ, _f, _s, _t, plen = HEADER.unpack(hdr)
        if plen:
            self.sock.recv(plen)
        if typ != TYPE_ACK:
            raise OSError("hello 失败")
        self._rx = threading.Thread(target=self._rx_loop, name="hid-rx", daemon=True)
        self._rx.start()

    def stick(self, v: float, w: float) -> None:
        payload = STICK.pack(
            int(max(-32000, min(32000, round(v * 1000)))),
            int(max(-32000, min(32000, round(w * 1000)))),
            0,
        )
        self._send(TYPE_STICK, payload)

    def estop(self) -> None:
        self._send(TYPE_ESTOP)


def main() -> int:
    parser = argparse.ArgumentParser(description="笔记本本机 → 工控机 :9100 遥控")
    parser.add_argument("--host", default="", help="工控机局域网 IP（不要填 127.0.0.1，除非就在工控机本机）")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--token", default=DEFAULT_TOKEN)
    parser.add_argument("--v", type=float, default=0.10)
    parser.add_argument("--w", type=float, default=0.40)
    parser.add_argument("--hz", type=float, default=100.0, help="采样频率（默认 100，老师目标）")
    args = parser.parse_args()

    if _on_jetson() or _ssh():
        ips = " ".join(_lan_ips()) or "<工控机局域网IP>"
        print(
            "你正在工控机 / Cursor 远程终端里运行本文件。\n"
            "这里的项目目录在 Jetson 硬盘上；笔记本只是远程看着它。\n"
            "请把 teleop_from_laptop.py 拷到笔记本，打开笔记本自己的 PowerShell\n"
            "（开始菜单搜 powershell，不是 Cursor 底部终端）执行：\n"
            f"  python teleop_from_laptop.py --host {ips.split()[0] if ips else '<工控机IP>'}\n"
            f"工控机当前地址: {ips}",
            file=sys.stderr,
        )
        return 2

    host = args.host.strip()
    if not host or host in ("127.0.0.1", "localhost"):
        print("请加 --host <工控机局域网IP>。在工控机 run_local 启动横幅里能看到。", file=sys.stderr)
        return 2

    try:
        stick = open_stick()
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 2

    hz = max(20.0, min(200.0, args.hz))
    period = 1.0 / hz
    print(f"HID={stick.backend}  →  tcp://{host}:{args.port}  {hz:.0f} Hz（stick 不等 ACK）")
    client = Client(host, args.port, args.token)
    try:
        client.hello(args.token)
        print("已连上工控机。点一下本窗口再按键。WASD 可同时按，SPACE 急停，+/- 点一下调一档，Q 退出。")
        v_step, w_step = args.v, args.w
        last = 0.0
        plus_prev = minus_prev = space_sent = False
        while True:
            t0 = time.monotonic()
            fr = stick.frame()
            if fr["q"]:
                client.estop()
                print("Q：退出")
                break
            if fr["space"]:
                if not space_sent:
                    client.estop()
                    space_sent = True
            else:
                space_sent = False
                if fr["plus"] and not plus_prev:
                    v_step = min(0.50, v_step + 0.01)
                    w_step = min(1.00, w_step + 0.02)
                if fr["minus"] and not minus_prev:
                    v_step = max(0.05, v_step - 0.01)
                    w_step = max(0.10, w_step - 0.02)
                client.stick(fr["v"] * v_step, fr["w"] * w_step)
            plus_prev, minus_prev = fr["plus"], fr["minus"]
            now = time.monotonic()
            if now - last > 0.4:
                last = now
                sys.stdout.write(
                    f"\r  v={fr['v']:+d} w={fr['w']:+d}  档={v_step:.2f}/{w_step:.2f}   "
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
