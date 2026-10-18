#!/usr/bin/env bash
# 把 TCP 遥操装成 systemd 服务（只开遥控、不跑完整 agent 时用）。
# 日常 mock_cloud + agent：不必装本服务，重启 agent 即可，9100 会自己起来。
set -euo pipefail
cd "$(dirname "$0")"
UNIT_SRC="$(pwd)/systemd/bunker-teleop.service"
sudo cp "$UNIT_SRC" /etc/systemd/system/bunker-teleop.service
sudo systemctl daemon-reload
echo "已安装 bunker-teleop.service"
echo "  启动: sudo systemctl enable --now bunker-teleop"
echo "  状态: systemctl status bunker-teleop"
echo "若 agent 已在跑，先: sudo systemctl stop bunker-teleop"
echo "笔记本: python3 teleop_tcp_client.py --host <工控机IP>"
