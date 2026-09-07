#!/usr/bin/env python3
"""Bunker 网页控制台桌面客户端（pywebview 封装）。

把工控机 ``run_local.py`` 提供的 ``http://<IP>:9101`` 装进独立窗口，
没有浏览器地址栏/标签页。页面本身不改，WebSocket / WASD 仍走原协议。

    python3 teleop_desktop.py
    python3 teleop_desktop.py --host 172.18.101.12

HOST 未指定时按 teleop_client.conf 的 CANDIDATE_HOSTS 探测 :9101
（当前校园网 172.18.101.12，并保留原遥控网 59.79.233.120）。
缺 pywebview 时自动退回系统浏览器。笔记本请用 ``teleop_client_laptop/``。
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CONF = ROOT / "teleop_client.conf"
DEFAULT_PORT = 9101
DEFAULT_FALLBACK = "59.79.233.120"
DEFAULT_CANDIDATES = ("172.18.101.12", "59.79.233.120")
WAIT_S = 6.0

_OFFLINE_HTML = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<title>Bunker 控制台</title>
<style>
  html, body {{
    margin: 0; min-height: 100%;
    background: radial-gradient(900px 420px at 20% -10%, #163044 0%, transparent 55%), #0a1016;
    color: #e8eef5;
    font: 15px/1.5 "Segoe UI", "PingFang SC", "Noto Sans SC", "Microsoft YaHei", sans-serif;
  }}
  .box {{
    max-width: 520px; margin: 12vh auto; padding: 28px 26px;
    background: #141d27; border: 1px solid #243140; border-radius: 16px;
  }}
  h1 {{ margin: 0 0 8px; font-size: 20px; }}
  p {{ color: #8b9aab; }}
  code {{ color: #5ec8ff; }}
  label {{ display: block; margin: 16px 0 6px; color: #8b9aab; font-size: 12px; }}
  input {{
    width: 100%; box-sizing: border-box; padding: 10px 12px;
    background: #0d141c; border: 1px solid #334556; border-radius: 8px;
    color: #e8eef5; font: inherit;
  }}
  button {{
    margin-top: 16px; width: 100%; padding: 10px 12px; border-radius: 8px;
    border: 1px solid #2d6f8f; background: #123346; color: #d7f3ff;
    font: inherit; cursor: pointer;
  }}
</style>
</head>
<body>
  <div class="box">
    <h1>连不上工控机网页</h1>
    <p>目标：<code>{url}</code></p>
    <p>将依次尝试：{hint}</p>
    <p>请确认工控机已运行 <code>python3 run_local.py</code>，且笔记本与工控机在同一局域网。</p>
    <label for="host">工控机 IP</label>
    <input id="host" value="{host}" />
    <label for="port">端口</label>
    <input id="port" value="{port}" />
    <button onclick="retry()">重新连接</button>
  </div>
  <script>
    function retry() {{
      const h = document.getElementById('host').value.trim();
      const p = document.getElementById('port').value.trim() || '9101';
      if (window.pywebview && window.pywebview.api) {{
        window.pywebview.api.reconnect(h, p);
      }}
    }}
  </script>
</body>
</html>
"""


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


def _save_host(host: str, port: int) -> None:
    lines: list[str] = []
    if CONF.is_file():
        lines = CONF.read_text(encoding="utf-8").splitlines()
    written = {"HOST": False, "PORT": False}
    out: list[str] = []
    for line in lines:
        raw = line.strip()
        if raw.startswith("HOST="):
            out.append(f"HOST={host}")
            written["HOST"] = True
        elif raw.startswith("PORT="):
            out.append(f"PORT={port}")
            written["PORT"] = True
        else:
            out.append(line)
    if not written["HOST"]:
        out.append(f"HOST={host}")
    if not written["PORT"]:
        out.append(f"PORT={port}")
    CONF.write_text("\n".join(out) + "\n", encoding="utf-8")


def _port_open(host: str, port: int, timeout: float = 0.4) -> bool:
    sock = socket.socket()
    sock.settimeout(timeout)
    try:
        sock.connect((host, port))
    except OSError:
        return False
    finally:
        sock.close()
    return True


def _split_hosts(*chunks: str) -> list[str]:
    seen: set[str] = set()
    out: list[str] = []
    for chunk in chunks:
        if not chunk:
            continue
        for tok in chunk.replace(",", " ").replace(";", " ").split():
            host = tok.strip()
            if not host or host in seen or host.startswith("192.168.1."):
                continue
            seen.add(host)
            out.append(host)
    return out


def _hint_html(hosts: list[str]) -> str:
    return " / ".join(f"<code>{h}</code>" for h in hosts) or "<code>（无候选）</code>"


def _candidate_hosts(configured: str, fallback: str, extra: str) -> list[str]:
    return _split_hosts(configured, extra, fallback, ",".join(DEFAULT_CANDIDATES))


def _pick_host(configured: str, fallback: str, port: int, extra: str = "") -> str:
    """Probe configured IP first, then CANDIDATE_HOSTS. Keep legacy 59.x."""
    hosts = _candidate_hosts(configured, fallback, extra)
    for host in hosts:
        if _port_open(host, port):
            return host
    return hosts[0] if hosts else DEFAULT_FALLBACK


def _wait_ready(host: str, port: int, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _port_open(host, port):
            return True
        time.sleep(0.3)
    return False


class _Api:
    def __init__(self, window_holder: dict) -> None:
        self._holder = window_holder

    def reconnect(self, host: str, port: str) -> None:
        host = (host or "").strip()
        try:
            port_i = int(port)
        except (TypeError, ValueError):
            port_i = DEFAULT_PORT
        if not host:
            return
        _save_host(host, port_i)
        window = self._holder.get("window")
        if window is None:
            return
        url = f"http://{host}:{port_i}"
        hint = _hint_html(_candidate_hosts(host, DEFAULT_FALLBACK, ",".join(DEFAULT_CANDIDATES)))
        if _wait_ready(host, port_i, 2.0):
            window.load_url(url)
        else:
            window.load_html(_OFFLINE_HTML.format(
                url=url, host=host, port=port_i, hint=hint,
            ))


def _open_browser(url: str) -> int:
    opened = webbrowser.open(url, new=2)
    print(f"已用系统浏览器打开 {url}" if opened else f"请手动打开 {url}")
    return 0 if opened else 1


def main() -> int:
    conf = _load_conf(CONF)
    parser = argparse.ArgumentParser(description="Bunker 网页控制台桌面客户端")
    parser.add_argument("--host", default=conf.get("HOST", ""), help="工控机 IP")
    parser.add_argument("--port", type=int, default=int(conf.get("PORT") or DEFAULT_PORT))
    parser.add_argument("--browser", action="store_true", help="强制用系统浏览器，不启 pywebview")
    args = parser.parse_args()

    fallback = conf.get("FALLBACK_HOST") or DEFAULT_FALLBACK
    extra = conf.get("CANDIDATE_HOSTS") or ""
    hosts = _candidate_hosts(args.host.strip(), fallback, extra)
    host = _pick_host(args.host.strip(), fallback, args.port, extra)
    url = f"http://{host}:{args.port}"
    ready = _wait_ready(host, args.port, WAIT_S)
    hint = _hint_html(hosts)

    if args.browser:
        return _open_browser(url)

    try:
        import webview
    except ImportError:
        print("未安装 pywebview，退回系统浏览器。笔记本可执行: pip install pywebview", file=sys.stderr)
        return _open_browser(url)

    holder: dict = {}
    window = webview.create_window(
        title="Bunker 控制台",
        url=url if ready else None,
        html=None if ready else _OFFLINE_HTML.format(
            url=url, host=host, port=args.port, hint=hint,
        ),
        width=1280,
        height=860,
        min_size=(900, 640),
        text_select=True,
        confirm_close=False,
        js_api=_Api(holder),
    )
    holder["window"] = window
    print(f"桌面客户端 → {url}" + ("" if ready else "（工控机未就绪，显示重连页）"))
    webview.start()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
