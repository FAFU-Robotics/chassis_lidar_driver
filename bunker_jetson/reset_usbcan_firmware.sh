#!/usr/bin/env bash
# ============================================================
# USB-CAN 适配器固件卡死恢复（cansend 成功但 TX/RX 全静默时用）
#
# 适用场景：
#   can1 已 UP、gs_usb 已绑定、cansend 返回 0，
#   但 TX 计数不涨、RX 完全静默、candump 无任何帧。
#   → USB-CAN 适配器固件卡死（USB URB 提交成功但固件不响应）。
#
# 恢复步骤（由轻到重）:
#   1. 结束占用设备的用户态程序
#   2. usbreset 软复位（/dev/bus/usb）
#   3. 卸载并重载 gs_usb 内核模块（强制固件重新初始化）
#   4. 重新拉起 can1
#   5. 验证：cansend + candump 自环
#
# 用法（需要 root）:
#   sudo bash reset_usbcan_firmware.sh
# ============================================================
set -uo pipefail

echo "=== 0. 前置确认 ==="
lsusb -d 1d50:606f >/dev/null 2>&1 || { echo "✗ 设备不在！请插入 USB-CAN"; exit 1; }
echo "  ✓ 设备存在: $(lsusb -d 1d50:606f)"

echo
echo "=== 1. 结束占用设备的用户态程序 ==="
pkill -f gs_usb 2>/dev/null || true
pkill -f GsUsb 2>/dev/null || true
pkill -f "run_agent" 2>/dev/null || true
pkill -f "can.reader" 2>/dev/null || true
sleep 1

echo
echo "=== 2. usbreset 软复位 ==="
BUS=$(lsusb -d 1d50:606f | awk '{print $2}')
ADDR=$(lsusb -d 1d50:606f | awk '{print $4}' | tr -d ':')
DEVNODE="/dev/bus/usb/$(printf '%03d' "$BUS")/$(printf '%03d' "$ADDR")"
echo "  复位 $DEVNODE ..."
if usbreset "$DEVNODE" 2>/dev/null; then
    echo "  ✓ usbreset 成功"
else
    echo "  △ usbreset 失败（设备节点不可见/无权限），继续下一步"
fi
sleep 2

echo
echo "=== 3. 卸载并重载 gs_usb 内核模块（强制固件重新初始化）==="
# 先确保没有 can 接口占用
for iface in /sys/class/net/can*; do
    [[ -e "$iface" ]] || continue
    name=$(basename "$iface")
    dev=$(readlink -f "$iface/device" 2>/dev/null || true)
    if [[ "$dev" == *"usb"* ]]; then
        echo "  先 down $name ..."
        ip link set "$name" down 2>/dev/null || true
    fi
done
if modprobe -r gs_usb 2>/dev/null; then
    echo "  ✓ gs_usb 已卸载"
    sleep 1
    modprobe gs_usb
    echo "  ✓ gs_usb 已重新加载（固件重新初始化）"
    sleep 2
else
    echo "  ✗ 无法卸载 gs_usb（可能被占用）。尝试强制... "
    echo "  请先关闭所有使用 CAN 的程序（agent/监控），再重试本脚本。"
fi

echo
echo "=== 4. 重新拉起 can1 ==="
CAN_USB=""
for iface in /sys/class/net/can*; do
    [[ -e "$iface" ]] || continue
    name=$(basename "$iface")
    dev=$(readlink -f "$iface/device" 2>/dev/null || true)
    if [[ "$dev" == *"usb"* ]]; then
        CAN_USB="$name"
        break
    fi
done
if [[ -n "$CAN_USB" ]]; then
    echo "  USB-CAN 网卡: $CAN_USB"
    ip link set "$CAN_USB" down 2>/dev/null || true
    ip link set "$CAN_USB" up type can bitrate 500000 restart-ms 100
    sleep 1
    echo "  ✓ 已拉起: $CAN_USB @ 500k"
    echo "  carrier=$(cat /sys/class/net/$CAN_USB/carrier 2>/dev/null)"
else
    echo "  ✗ 重载后仍无 USB-CAN 网卡！"
    echo "  请【物理拔掉 USB-CAN，等 3 秒，再插回】后重跑本脚本。"
    exit 2
fi

echo
echo "=== 5. 验证（自环：发送后应能收回 echo 帧）==="
TX0=$(cat /sys/class/net/$CAN_USB/statistics/tx_packets 2>/dev/null || echo 0)
timeout 3 candump -n 3 "$CAN_USB" > /tmp/fw_check.txt 2>&1 &
DUMP=$!
sleep 1
cansend "$CAN_USB" 123#DEADBEEF >/dev/null 2>&1
cansend "$CAN_USB" 456#AABBCCDD >/dev/null 2>&1
sleep 1
kill $DUMP 2>/dev/null; wait $DUMP 2>/dev/null
TX1=$(cat /sys/class/net/$CAN_USB/statistics/tx_packets 2>/dev/null || echo 0)
echo "  TX 计数: $TX0 → $TX1"
if grep -q '^  ' /tmp/fw_check.txt 2>/dev/null && [[ -s /tmp/fw_check.txt ]]; then
    echo "  ✓ 收回 echo 帧 —— 固件已恢复正常！"
    cat /tmp/fw_check.txt | sed 's/^/    /'
    echo
    echo "  ✅ 适配器恢复正常。可以启动 agent:"
    echo "     export BUNKER_CAN_CHANNEL=$CAN_USB && bash start_agent.sh"
elif [[ "$TX1" -gt "$TX0" ]]; then
    echo "  △ TX 计数增加了但没收回 echo（总线无其他节点，或需再验证）"
else
    echo "  ✗ TX 计数仍不涨 —— 固件可能仍卡死。"
    echo "  【最终手段】请【物理拔掉 USB-CAN 适配器，等 3 秒，再插回】，"
    echo "  然后重跑: sudo bash reset_usbcan_firmware.sh"
fi
