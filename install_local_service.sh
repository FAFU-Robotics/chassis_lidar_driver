#!/usr/bin/env bash
# 安装并（默认）启用 bunker-local.service：开机后台开 :9100 + :9101。
# 不开雷达驱动、不弹桌面窗。笔记本用 teleop_client_laptop/ 双击打开网页。
#
# 用法（仓库根目录，需要 sudo）：
#   bash install_local_service.sh
#   bash install_local_service.sh --no-start    # 只安装，不立刻启动
#   bash install_local_service.sh --disable     # 停止并取消开机自启
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
UNIT_SRC="$ROOT/bunker_jetson/systemd/bunker-local.service"
UNIT_DST=/etc/systemd/system/bunker-local.service
START=1
DISABLE=0

for arg in "$@"; do
  case "$arg" in
    --no-start) START=0 ;;
    --disable) DISABLE=1 ;;
    -h|--help)
      grep -E '^#' "$0" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *)
      echo "未知参数: $arg" >&2
      exit 2
      ;;
  esac
done

if [[ ! -f "$UNIT_SRC" ]]; then
  echo "找不到 $UNIT_SRC" >&2
  exit 1
fi

if [[ "$DISABLE" == 1 ]]; then
  sudo systemctl disable --now bunker-local.service 2>/dev/null || true
  echo "已停止并取消开机自启 bunker-local"
  exit 0
fi

PY="$(command -v python3 || true)"
if [[ -x /home/fafu_robot/miniconda3/bin/python3 ]]; then
  PY=/home/fafu_robot/miniconda3/bin/python3
fi
if [[ -z "$PY" ]]; then
  echo "找不到 python3" >&2
  exit 1
fi

echo "使用 Python: $PY"
echo "仓库根目录: $ROOT"

# 若旧的只开 :9100 的服务在跑，先停掉，避免抢端口
if systemctl is-active --quiet bunker-teleop.service 2>/dev/null; then
  echo "检测到 bunker-teleop.service 正在运行，将先停止（与本服务抢 :9100）"
  sudo systemctl disable --now bunker-teleop.service || true
fi

TMP="$(mktemp)"
sed \
  -e "s|^ExecStart=.*|ExecStart=$PY $ROOT/run_local.py --daemon --no-lidar|" \
  -e "s|^WorkingDirectory=.*|WorkingDirectory=$ROOT|" \
  "$UNIT_SRC" > "$TMP"

sudo cp "$TMP" "$UNIT_DST"
rm -f "$TMP"
sudo systemctl daemon-reload

echo "已安装 $UNIT_DST"

if [[ "$START" == 1 ]]; then
  if ss -ltn 2>/dev/null | grep -q ':9100 '; then
    echo "⚠ 端口 9100 已被占用。请先结束手动的 run_local.py / run_agent.py，再执行："
    echo "    sudo systemctl start bunker-local"
    echo "仍会 enable 开机自启。"
    sudo systemctl enable bunker-local.service
  else
    sudo systemctl enable --now bunker-local.service
    echo "已 enable 并启动 bunker-local"
  fi
else
  sudo systemctl enable bunker-local.service
  echo "已 enable（未立即 start）。启动: sudo systemctl start bunker-local"
fi

echo
echo "常用："
echo "  systemctl status bunker-local"
echo "  journalctl -u bunker-local -e"
echo "  sudo systemctl restart bunker-local"
echo "笔记本：拷贝 teleop_client_laptop/ ，双击 打开网页控制台.bat"
echo "雷达默认关，网页里再点「雷达开」。不要同时再跑 python3 run_local.py"
