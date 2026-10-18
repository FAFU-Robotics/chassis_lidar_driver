#!/usr/bin/env bash
# ============================================================
# CAN 终极定位：底盘到底在哪个口 / 是否真的在线
#
# 用法（需要 root）:
#   sudo bash find_chassis.sh
#
# 原理：
#   BUNKER MINI 底盘只要【开机】并接在总线上，就会每 ~10ms
#   持续广播 0x211（系统状态）。本脚本同时监听 can0 和 can1，
#   哪个口 15 秒内有 0x211，底盘就在哪个口。
#   （先用 cansend 发帧确认适配器自身能收发）
# ============================================================
set -uo pipefail

echo "=== 1. 所有 can 网卡状态 ==="
for iface in /sys/class/net/can*; do
    [[ -e "$iface" ]] || continue
    name=$(basename "$iface")
    dev=$(readlink -f "$iface/device" 2>/dev/null || true)
    kind="USB-CAN"; [[ "$dev" == *"mttcan"* ]] && kind="板载 mttcan"
    echo "  $name: carrier=$(cat $iface/carrier 2>/dev/null)  operstate=$(cat $iface/operstate 2>/dev/null)  ($kind)"
done

echo
echo "=== 2. 适配器自环验证（cansend + candump）==="
CH_TEST=""
for iface in /sys/class/net/can*; do
    [[ -e "$iface" ]] || continue
    name=$(basename "$iface")
    dev=$(readlink -f "$iface/device" 2>/dev/null || true)
    if [[ "$dev" == *"usb"* ]]; then CH_TEST="$name"; break; fi
done
if [[ -n "$CH_TEST" ]]; then
    echo "  用 $CH_TEST 做自环测试："
    timeout 3 candump -n 3 "$CH_TEST" > /tmp/find_chassis_echo.txt 2>&1 &
    DP=$!
    sleep 0.5
    cansend "$CH_TEST" 5A5#CAFEBABE >/dev/null 2>&1
    sleep 1
    kill $DP 2>/dev/null; wait $DP 2>/dev/null
    if [[ -s /tmp/find_chassis_echo.txt ]]; then
        echo "  ✓ $CH_TEST 能收回 echo —— 适配器收发正常！"
    else
        echo "  ✗ $CH_TEST 收不到自己的 echo —— 适配器可能卡死，建议物理拔插。"
    fi
fi

echo
echo "=== 3. 同时监听 can0 和 can1（各 15 秒，找 0x211）==="
echo "  （底盘开机时会在所在口持续广播 0x211/0x221/0x311/0x361）"
FOUND=""
for ch in can0 can1; do
    [[ -e "/sys/class/net/$ch" ]] || { echo "  $ch 不存在，跳过"; continue; }
    echo "  --- $ch (15s) ---"
    # 注意: candump 每行有前导空格，不能锚定 ^。用 awk 取第 2 列(帧 ID)判断。
    if timeout 15 candump -n 8 "$ch" 2>&1 | tee /tmp/find_$ch.txt \
        | awk '{print $2}' | grep -qx '211'; then
        FOUND="$ch"
        echo "  ✓ $ch 收到 0x211 —— 底盘在这个口！"
        break
    fi
    echo "  （15 秒无 0x211）"
done

echo
echo "=== 4. 结论 ==="
if [[ -n "$FOUND" ]]; then
    echo "  ✅ 底盘在 $FOUND 上！链路正常！"
    echo "  启动 agent:"
    echo "    export BUNKER_CAN_CHANNEL=$FOUND && bash start_agent.sh"
elif [[ -s /tmp/find_can1.txt ]]; then
    echo "  △ can1 有 CAN 流量但没有 0x211（可能是别的设备/波特率不符）"
    echo "  收到的帧:"
    cat /tmp/find_can1.txt | sed 's/^/    /' | head -8
    echo "  若帧 ID 都是别的值，说明底盘波特率不是 500K，或接的不是这个口。"
else
    echo "  ✗ can0 和 can1 都收不到 0x211。"
    echo
    echo "  【决定性结论】底盘【没有】在 CAN 总线上广播。"
    echo "  原因只可能是以下之一："
    echo "    1. 底盘没开机（指示灯/风扇有没有反应？）"
    echo "    2. CAN_H/CAN_L 没接到【底盘 CAN 口】或接反"
    echo "    3. 适配器固件卡死（需要物理拔插 USB-CAN）"
    echo "    4. 波特率不是 500K"
    echo
    echo "  请先确认 1 和 2 —— 这是 90% 的情况。"
    echo "  若确认底盘开机且接线正确，物理拔插适配器后重跑本脚本。"
fi
