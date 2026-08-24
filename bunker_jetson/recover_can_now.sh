#!/usr/bin/env bash
# ============================================================
# 立即恢复 USB-CAN 网卡（设备枚举正常但没生成 can* 网卡时用）
#
# 适用场景：
#   lsusb 能看到 1d50:606f（设备枚举正常），但 /sys/class/net
#   下没有 USB 归属的 can 网卡（gs_usb 驱动未绑定），只剩板载
#   can0 —— 此时 agent 永远收不到底盘反馈，小车不动。
#
# 原因：设备插入时 probe 时机错过 / 设备被用户态程序卡住
#       （例如直接打开过 gs_usb/libusb 设备后再没释放）。
#
# 用法（需要 root）:
#   sudo bash recover_can_now.sh
# ============================================================
set -euo pipefail

echo "=== 1. 确认设备 ==="
if ! lsusb -d 1d50:606f >/dev/null 2>&1; then
    echo "  ✗ 设备未被枚举！请【物理拔掉 USB-CAN，等 3 秒，再插回】后重跑。"
    exit 1
fi
echo "  ✓ $(lsusb -d 1d50:606f)"

echo
echo "=== 2. 结束可能占用设备的用户态程序 ==="
pkill -f gs_usb 2>/dev/null || true
pkill -f GsUsb 2>/dev/null || true
pkill -f python-can 2>/dev/null || true
sleep 1

echo
echo "=== 3. usbreset 软复位（触发重新枚举/重新绑定）==="
BUS=$(lsusb -d 1d50:606f | awk '{print $2}')
ADDR=$(lsusb -d 1d50:606f | awk '{print $4}' | tr -d ':')
DEVNODE="/dev/bus/usb/$(printf '%03d' "$BUS")/$(printf '%03d' "$ADDR")"
echo "  复位: $DEVNODE"
if usbreset "$DEVNODE" 2>/dev/null; then
    echo "  ✓ usbreset 成功"
    sleep 3
else
    echo "  △ usbreset 不可用/失败，尝试 authorized 0/1..."
    DEVPATH=""
    for d in /sys/bus/usb/devices/*; do
        [[ -f "$d/idVendor" ]] || continue
        if grep -qi '^1d50$' "$d/idVendor" && grep -qi '^606f$' "$d/idProduct"; then
            DEVPATH="$d"
            break
        fi
    done
    if [[ -n "$DEVPATH" ]]; then
        echo 0 > "$DEVPATH/authorized" 2>/dev/null || true
        sleep 2
        echo 1 > "$DEVPATH/authorized" 2>/dev/null || true
        sleep 3
    fi
fi

echo
echo "=== 4. 确认 USB-CAN 是否已注册为 can 网卡 ==="
CAN_USB=""
for iface in /sys/class/net/can*; do
    [[ -e "$iface" ]] || continue
    dev=$(readlink -f "$iface/device" 2>/dev/null || true)
    if [[ "$dev" == *"usb"* ]]; then
        CAN_USB=$(basename "$iface")
        break
    fi
done

if [[ -n "$CAN_USB" ]]; then
    echo "  ✓ USB-CAN 已注册为网卡: $CAN_USB"
    ip link set "$CAN_USB" down 2>/dev/null || true
    ip link set "$CAN_USB" up type can bitrate 500000 restart-ms 100
    echo "  ✓ 已拉起: $CAN_USB @ 500k"
    echo
    echo "  === 5. 探测底盘反馈（0x211）==="
    if timeout 3 candump -n 5 "$CAN_USB" 2>/dev/null | grep -q '211'; then
        echo "  ✓ 收到底盘反馈帧 0x211 —— 底盘在这个口！"
    else
        echo "  △ 此口 3 秒内无 0x211（检查底盘上电 / CAN_H/CAN_L 接线 / 波特率）"
    fi
    echo
    echo "  底盘应该在: $CAN_USB"
    echo "  启动 agent（建议显式指定）:"
    echo "    export BUNKER_CAN_CHANNEL=$CAN_USB && bash start_agent.sh"
else
    echo "  ✗ 仍未生成 can 网卡。"
    echo
    echo "  【最终手段】请【物理拔掉 USB-CAN 适配器，等 3 秒，再插回】，"
    echo "  然后重新运行: sudo bash recover_can_now.sh"
    echo "  再不行则检查: 换一个 USB 口 / 检查 gs_usb 内核模块是否加载"
    echo "    lsmod | grep gs_usb   # 应显示已加载"
    echo "    sudo modprobe gs_usb   # 未加载则手动加载"
    exit 2
fi
