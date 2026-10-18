#!/usr/bin/env python3
"""打开 Bunker 网页控制台（浏览器客户端）。

工控机先跑 ``python3 run_local.py``（网页在 :9101）。本脚本只开浏览器，
不抢 :9100、不拉 agent。

    工控机本机双击  start_teleop_client.sh  /  桌面「打开网页控制台」
    笔记本双击      start_teleop_client.bat
    或命令行        python3 open_teleop_web.py
                    python3 open_teleop_web.py --host 59.79.233.120

地址优先读同目录 ``teleop_client.conf``；未写 HOST 时自动探测本机 :9101，
再回退到配置里的 FALLBACK_HOST。
"""

from __future__ import annotations

import argparse
import os
import socket
import subprocess
import sys
import time
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CONF = ROOT / "teleop_client.conf"
DEFAULT_PORT = 9101
DEFAULT_FALLBACK = "59.79.233.120"
WAIT_S = 8.0


def _load_conf(path: Path) -> dict[str, str]:
    data: dict[str, str] = {}
    if not path.is_file():
        return data
    with path.open("r", encoding="utf-8") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, value = line.partition("=")
            data[key.strip()] = value.strip()
    return data


def _port_open(host: str, port: int, timeout: float = 0.35) -> bool:
    sock = socket.socket()
    sock.settimeout(timeout)
    try:
        sock.connect((host, port))
    except OSError:
        return False
    finally:
        sock.close()
    return True


def _lan_ips() -> list[str]:
    ips: list[str] = []
    try:
        for tok in os.popen("hostname -I 2>/dev/null").read().split():
            if tok.count(".") == 3 and not tok.startswith("127."):
                ips.append(tok)
    except Exception:
        pass
    return ips


def _alert(title: str, message: str, *, error: bool = True) -> None:
    print(message, file=sys.stderr if error else sys.stdout)
    if sys.platform.startswith("win"):
        try:
            import ctypes

            ctypes.windll.user32.MessageBoxW(
                0, message, title, 0x10 if error else 0x40,
            )
        except Exception:
            pass
        return
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        if error:
            cmds = (
                ["zenity", "--error", "--title", title, "--text", message, "--no-wrap"],
                ["notify-send", "--urgency=critical", title, message],
            )
        else:
            cmds = (
                ["notify-send", title, message],
            )
        for cmd in cmds:
            try:
                subprocess.call(cmd, timeout=8)
                break
            except Exception:
                continue


def _pick_host(configured: str, fallback: str, port: int) -> str:
    if configured:
        return configured
    if _port_open("127.0.0.1", port):
        return "127.0.0.1"
    for ip in _lan_ips():
        if _port_open(ip, port):
            return ip
    return fallback


def _wait_ready(host: str, port: int, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _port_open(host, port):
            return True
        time.sleep(0.35)
    return False


def main() -> int:
    conf = _load_conf(CONF)
    parser = argparse.ArgumentParser(description="打开 Bunker 网页控制台（浏览器客户端）")
    parser.add_argument(
        "--host",
        default=conf.get("HOST", ""),
        help="工控机 IP；默认读 teleop_client.conf，空则自动探测",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(conf.get("PORT") or DEFAULT_PORT),
        help="网页端口（默认 9101）",
    )
    parser.add_argument(
        "--no-wait",
        action="store_true",
        help="不探测、不等待，直接打开浏览器",
    )
    args = parser.parse_args()

    fallback = conf.get("FALLBACK_HOST") or DEFAULT_FALLBACK
    host = _pick_host(args.host.strip(), fallback, args.port)
    url = f"http://{host}:{args.port}"

    if not args.no_wait and not _wait_ready(host, args.port, WAIT_S):
        _alert(
            "Bunker 网页控制台",
            "连不上网页控制台：\n"
            f"  {url}\n\n"
            "请先在工控机仓库根目录运行：\n"
            "  python3 run_local.py\n\n"
            "笔记本请把 teleop_client.conf 里的 HOST 改成工控机局域网 IP。",
        )
        return 2

    opened = webbrowser.open(url, new=2)
    if not opened:
        _alert("Bunker 网页控制台", f"已找到服务，但打不开浏览器。请手动访问：\n{url}")
        return 1
    print(f"已打开 {url}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
