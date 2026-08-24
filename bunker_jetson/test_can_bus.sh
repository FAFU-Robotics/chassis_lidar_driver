#!/usr/bin/env bash
# ============================================================
# CAN 总线节点存在性测试（决定性验证）
#
# 原理：CAN 协议要求发送帧必须有【至少一个其他节点】在总线 ACK 位
# 应答。如果总线上只有适配器自己，cansend 会报 NO ACK 错误。
#
# 用法（需要 root）:
#   sudo bash test_can_bus.sh
# ============================================================
set -uo pipefail

echo "=== 0. 前置：can1 状态 ==="
echo "  operstate=$(cat /sys/class/net/can1/operstate 2>/dev/null)"
echo "  carrier=$(cat /sys/class/net/can1/carrier 2>/dev/null)"
echo "  bitrate=$(ip -details link show can1 2>/dev/null | grep -o 'bitrate [0-9]*')"
echo

echo "=== 1. 关键测试：cansend 详细输出 ==="
echo "  （若总线上没有其他节点，cansend 会报 NO ACK 并返回非 0）"
cansend can1 123#DEADBEEF
RC=$?
echo "  cansend exit=$RC"
if [[ $RC -eq 0 ]]; then
    echo "  ✓ 发送成功 → 总线【有节点在应答 ACK】！"
    echo "    （如果底盘没接，总线上没有节点，会报 NO ACK 错误）"
    echo "    → 说明适配器与总线物理连接正常，且总线上存在另一个节点。"
else
    echo "  ✗ 发送失败/NO ACK → 总线上【没有其他节点】。"
    echo "    → 底盘没开机，或 CAN_H/CAN_L 根本没接上底盘。"
fi
echo

echo "=== 2. 连续发送 5 帧并观察统计 ==="
TX0=$(cat /sys/class/net/can1/statistics/tx_packets 2>/dev/null || echo 0)
for i in 1 2 3 4 5; do
    cansend can1 0${i}23#0102030405060708 >/dev/null 2>&1
done
sleep 1
TX1=$(cat /sys/class/net/can1/statistics/tx_packets 2>/dev/null || echo 0)
RX1=$(cat /sys/class/net/can1/statistics/rx_packets 2>/dev/null || echo 0)
echo "  TX 计数: $TX0 → $TX1  （发送 5 帧后应 +5）"
echo "  RX 计数: $RX1"
echo
if [[ "$TX1" -gt "$TX0" ]]; then
    echo "  ✓ TX 计数增加了 $((TX1 - TX0)) —— 帧确实发上了总线"
else
    echo "  △ TX 计数未增加 —— 帧未真正上总线（适配器/驱动异常）"
fi
echo

echo "=== 3. 长监听（10 秒）看底盘是否广播 ==="
echo "  BUNKER MINI 上电后每 ~10ms 广播 0x211，10 秒应有约 1000 帧"
timeout 10 candump -n 20 can1 2>&1 | head -25
echo "  (以上 10 秒收到的帧)"
echo

echo "=== 4. 结论 ==="
RX2=$(cat /sys/class/net/can1/statistics/rx_packets 2>/dev/null || echo 0)
if [[ "$RX2" -gt 0 ]]; then
    echo "  ✓ can1 总共有 $RX2 个 RX 包 —— 底盘在广播！链路 OK！"
    echo "  可以直接启动 agent: export BUNKER_CAN_CHANNEL=can1 && bash start_agent.sh"
else
    echo "  ✗ can1 RX 仍为 0 —— 底盘没有广播任何帧。"
    echo
    echo "  【排查方向】"
    if [[ $RC -eq 0 ]]; then
        echo "   • cansend 成功说明总线上有节点 ACK，但 RX=0。"
        echo "   • 可能: 底盘在广播但波特率不匹配（换 250K 试试?）"
        echo "   • 可能: ACK 来自适配器内部回环，底盘其实没接。"
    else
        echo "   • cansend NO ACK 说明总线上只有适配器自己。"
        echo "   • 底盘未开机 或 CAN_H/CAN_L 未接到底盘。"
    fi
    echo "   • 请检查底盘电源、CAN_H/CAN_L 接线方向"
fi
