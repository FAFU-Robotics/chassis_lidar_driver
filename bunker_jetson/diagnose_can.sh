#!/usr/bin/env bash
# ============================================================
# CAN 链路最终诊断（can1 已恢复但怀疑底盘不在线上时用）
#
# 用法（需要 root）:
#   sudo bash diagnose_can.sh
#
# 依次输出：
#   1. can 网卡状态与 USB 归属
#   2. gs_usb 驱动绑定
#   3. can1 主动发送 + 自环验证（看适配器能否收发）
#   4. can1 / can0 各监听 5 秒抓包
#   5. 接口收发统计
#   6. 结论与建议
# ============================================================
set -uo pipefail

echo "=== 1. CAN 网卡状态 ==="
for iface in /sys/class/net/can*; do
    [[ -e "$iface" ]] || continue
    name=$(basename "$iface")
    dev=$(readlink -f "$iface/device" 2>/dev/null || true)
    kind="USB-CAN" ; [[ "$dev" == *"mttcan"* ]] && kind="板载 mttcan"
    carrier=$(cat "$iface/carrier" 2>/dev/null)
    echo "  $name: operstate=$(cat $iface/operstate 2>/dev/null)  carrier=$carrier  ($kind)"
done

echo
echo "=== 2. gs_usb 驱动绑定 ==="
ls /sys/bus/usb/drivers/gs_usb/ 2>/dev/null | grep -E '^[0-9]+-' || echo "  (gs_usb 无绑定接口)"

echo
echo "=== 3. can1 主动发送 + 自环验证 ==="
# 先清统计，后台开 candump 监听自己的发送
TX0=$(cat /sys/class/net/can1/statistics/tx_packets 2>/dev/null || echo 0)
timeout 5 candump -n 3 can1 > /tmp/diag_can_echo.txt 2>&1 &
DUMP_PID=$!
sleep 1
# 连发几帧（不同 ID）
for id in 7FF 123 456; do
    cansend can1 ${id}#DEADBEEF 2>&1
    echo "  cansend ${id} exit=$?"
    sleep 0.3
done
sleep 1
kill $DUMP_PID 2>/dev/null
wait $DUMP_PID 2>/dev/null
TX1=$(cat /sys/class/net/can1/statistics/tx_packets 2>/dev/null || echo 0)
RX1=$(cat /sys/class/net/can1/statistics/rx_packets 2>/dev/null || echo 0)
echo "  TX 计数: $TX0 → $TX1   RX 计数: $RX1"
if grep -q '^  ' /tmp/diag_can_echo.txt 2>/dev/null && [[ -s /tmp/diag_can_echo.txt ]]; then
    echo "  ✓ 收回了自己发送的帧（适配器收发正常）:"
    sed 's/^/    /' /tmp/diag_can_echo.txt
else
    echo "  ✗ 未能收回自己发送的帧 —— 适配器自环失败！"
    echo "    （请将 USB-CAN 的 CAN_H 与 CAN_L 短接后重跑，确认适配器本身是否正常）"
fi

echo
echo "=== 4. 监听抓包（每个口 8 秒）==="
for ch in can1 can0; do
    echo "  --- $ch (8s) ---"
    timeout 8 candump -n 10 "$ch" 2>&1 | sed 's/^/    /'
    echo "     (以上 8 秒内收到的帧)"
done

echo
echo "=== 5. 接口收发统计 ==="
ip -details -s link show can1 2>&1 | tail -6

echo
echo "=== 6. 结论 ==="
# 用 python-can 再确认一次（更可靠）
HAS_FRAME=""
for ch in can1 can0; do
    if timeout 4 python3 -c "
import can, time
try:
    bus = can.interface.Bus(channel='$ch', interface='socketcan')
    end = time.time() + 3
    got = False
    while time.time() < end:
        if bus.recv(timeout=0.5) is not None:
            got = True
            break
    bus.shutdown()
    exit(0 if got else 1)
except Exception:
    exit(2)
" 2>/dev/null; then
        HAS_FRAME="$ch"
        break
    fi
done

if [[ -n "$HAS_FRAME" ]]; then
    echo "  ✓ $HAS_FRAME 收到 CAN 帧 —— 总线上有流量！"
elif [[ -s /tmp/diag_can_echo.txt ]]; then
    echo "  ✓ 适配器自环正常（能收发），但 can1/can0 上无底盘帧 —— 请检查底盘与适配器的接线。"
else
    echo "  ✗ can1 与 can0 均未收到任何 CAN 帧，且适配器自环也失败。"
    echo
    echo "  【结论】适配器本身可能有问题（驱动/固件/供电），或接线未通。"
    echo "  请逐项排查："
    echo "   1. 底盘是否【已开机】？指示灯/风扇有无反应？"
    echo "   2. USB-CAN 适配器的 CAN_H / CAN_L 是否接到【底盘 CAN 口】？"
    echo "      （CAN_H↔CAN_H、CAN_L↔CAN_L，接反/松动都会静默）"
    echo "   3. 短接适配器 CAN_H/CAN_L 重跑本脚本，验证适配器自身收发"
    echo "   4. 换一根 CAN 线 / 换一个 USB 口 / 给适配器供电"
fi
