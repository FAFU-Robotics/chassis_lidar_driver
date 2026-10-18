#!/usr/bin/env python3
"""停掉工控机上的旧控制台进程（:9100 代理 + :9101 网页）。

笔记本里的 teleop_client_laptop 只是一个浏览器窗口，关掉它不会换掉
工控机里已经加载的旧代码。页面改了却还是旧行为时，在工控机仓库根目录运行：

    python3 stop_local.py
    python3 stop_local.py --restart

只结束本仓库的 run_local.py / run_agent.py / teleop_web.py /
teleop_tcp_server.py，以及 bunker-local、bunker-teleop 这两个服务。
"""

from __future__ import annotations

import argparse
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
UNITS = ("bunker-local.service", "bunker-teleop.service")
PORTS = (9100, 9101)
_SCRIPT_NAMES = (
    "run_local.py",
    "run_agent.py",
    "teleop_web.py",
    "teleop_tcp_server.py",
)


def _mentions_script(text: str, name: str) -> bool:
    return re.search(rf"(^|[\s/]){re.escape(name)}(\s|$)", text) is not None


def cmdline_is_local_console(cmd: str, cwd: str, root: Path) -> bool:
    """True when this process is this repo's console, not some other Python."""
    text = cmd.replace("\x00", " ")
    root_s = str(root)
    if any(_mentions_script(text, str(root / name)) for name in ("run_local.py", "teleop_web.py")):
        return True
    jetson = root / "bunker_jetson"
    if any(_mentions_script(text, str(jetson / name)) for name in ("run_agent.py", "teleop_tcp_server.py")):
        return True
    # 相对路径启动：python3 run_agent.py，工作目录在本仓库或 bunker_jetson。
    cwd_ok = cwd == root_s or cwd.startswith(root_s + os.sep)
    if not cwd_ok:
        return False
    return any(_mentions_script(text, name) for name in _SCRIPT_NAMES)


def _read_proc(pid: int) -> tuple[str, str] | None:
    base = Path("/proc") / str(pid)
    try:
        raw = (base / "cmdline").read_bytes()
        if not raw.strip(b"\x00"):
            return None
        cmd = raw.replace(b"\x00", b" ").decode("utf-8", "replace").strip()
        cwd = os.readlink(base / "cwd")
    except OSError:
        return None
    return cmd, cwd


def find_console_pids(root: Path = ROOT) -> list[tuple[int, str]]:
    me = os.getpid()
    parent = os.getppid()
    found: list[tuple[int, str]] = []
    proc = Path("/proc")
    if not proc.is_dir():
        return found
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid in (me, parent):
            continue
        info = _read_proc(pid)
        if info is None:
            continue
        cmd, cwd = info
        if cmdline_is_local_console(cmd, cwd, root):
            found.append((pid, cmd))
    found.sort()
    return found


def _systemctl(args: list[str]) -> tuple[int, str]:
    last = "systemctl 不可用"
    for prefix in (["systemctl"], ["sudo", "-n", "systemctl"]):
        try:
            proc = subprocess.run(
                prefix + args,
                capture_output=True,
                text=True,
                timeout=25,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            last = str(exc)
            continue
        text = ((proc.stdout or "") + (proc.stderr or "")).strip()
        if proc.returncode == 0:
            return 0, text
        last = text or f"exit {proc.returncode}"
        if "password is required" in last or "Interactive authentication" in last:
            continue
        if "not found" in last.lower() or "could not be found" in last.lower():
            return proc.returncode, last
    return 1, last


def stop_services() -> list[str]:
    notes: list[str] = []
    for unit in UNITS:
        code, _ = _systemctl(["is-active", "--quiet", unit])
        if code != 0:
            continue
        stop_code, detail = _systemctl(["stop", unit])
        if stop_code == 0:
            notes.append(f"已停止服务 {unit}")
        else:
            notes.append(
                f"没能停止 {unit}（{detail or '需要密码'}）。"
                f"请自行执行: sudo systemctl stop {unit}"
            )
    return notes


def _signal_pids(pids: list[int], sig: int) -> None:
    for pid in pids:
        try:
            os.kill(pid, sig)
        except OSError:
            pass


def _still_alive(pids: list[int]) -> list[int]:
    alive = []
    for pid in pids:
        try:
            os.kill(pid, 0)
        except OSError:
            continue
        alive.append(pid)
    return alive


def stop_processes(root: Path = ROOT) -> list[str]:
    notes: list[str] = []
    targets = find_console_pids(root)
    if not targets:
        notes.append("没有发现本仓库的 run_local / run_agent / 网页进程")
        return notes
    for pid, cmd in targets:
        short = cmd if len(cmd) <= 140 else cmd[:137] + "..."
        notes.append(f"结束 PID {pid}: {short}")
    pids = [pid for pid, _ in targets]
    _signal_pids(pids, signal.SIGINT)
    time.sleep(1.2)
    left = _still_alive(pids)
    if left:
        _signal_pids(left, signal.SIGTERM)
        time.sleep(1.0)
        left = _still_alive(left)
    if left:
        _signal_pids(left, signal.SIGKILL)
        time.sleep(0.3)
        left = _still_alive(left)
    if left:
        notes.append("这些 PID 还在，可能需要 sudo: " + ", ".join(str(p) for p in left))
    else:
        notes.append("旧进程已退出")
    return notes


def ports_open(ports: tuple[int, ...] = PORTS) -> list[int]:
    import socket
    busy = []
    for port in ports:
        sock = socket.socket()
        sock.settimeout(0.3)
        try:
            if sock.connect_ex(("127.0.0.1", port)) == 0:
                busy.append(port)
        finally:
            sock.close()
    return busy


def restart_service() -> str:
    code, detail = _systemctl(["start", "bunker-local.service"])
    if code == 0:
        return "已重新启动 bunker-local（新代码会在这次进程里加载）"
    return (
        "没能自动拉起服务。"
        f"{detail}\n"
        "可手动执行:\n"
        "  sudo systemctl start bunker-local\n"
        f"  或: python3 {ROOT / 'run_local.py'} --daemon --no-lidar"
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="停掉工控机上的旧 Bunker 控制台进程")
    parser.add_argument(
        "--restart",
        action="store_true",
        help="停掉旧进程后立刻重新启动 bunker-local，让网页用上刚改的代码",
    )
    args = parser.parse_args(argv)
    print("笔记本窗口只是浏览器。旧程序在工控机上，正在清理本仓库的控制台进程。")
    for line in stop_services():
        print(line)
    # 服务若带 Restart=on-failure，先停服务再杀残留，避免刚杀掉又被拉起来。
    time.sleep(0.4)
    for line in stop_processes():
        print(line)
    busy = ports_open()
    if busy:
        print("这些端口仍被占用: " + ", ".join(str(p) for p in busy))
        print("若上面提示需要 sudo，执行后再跑一次本脚本。")
        return 1
    print("9100 和 9101 已经空出来。")
    if args.restart:
        print(restart_service())
        print("然后在笔记本上关掉控制台窗口，再双击一次「打开网页控制台.bat」。")
    else:
        print("需要再开时，在工控机执行: python3 stop_local.py --restart")
        print("或: sudo systemctl start bunker-local")
    return 0


if __name__ == "__main__":
    sys.exit(main())
