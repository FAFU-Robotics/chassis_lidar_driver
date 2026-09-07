#!/usr/bin/env bash
# 双击打开 Bunker 网页控制台（浏览器客户端）。工控机需已运行 python3 run_local.py。
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

if command -v python3 >/dev/null 2>&1; then
  PY=python3
elif command -v python >/dev/null 2>&1; then
  PY=python
else
  msg="找不到 python3。请先安装 Python，或在终端运行：python3 teleop_desktop.py"
  echo "$msg" >&2
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title "Bunker 网页控制台" --text "$msg" --no-wrap || true
  fi
  exit 1
fi

exec "$PY" "$HERE/teleop_desktop.py" "$@"
