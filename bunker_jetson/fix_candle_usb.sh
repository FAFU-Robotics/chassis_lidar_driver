#!/usr/bin/env bash
# ============================================================
# 一键修复 USB-CAN (candleLight / gs_usb) 无法使用的问题
#
# 背景：
#   Jetson 上 USB-CAN 适配器通过内核 gs_usb 驱动注册为 can* 网卡。
#   常见故障是「适配器插上了、lsusb 也能看到 1d50:606f，但：
#     1. /etc/udev/rules.d/99-gs_usb-candle.rules 语法错误
#        （行尾多了个右括号），导致整个规则文件解析失败，
#        MODE=0666 权限和「自动 ip link set up」全部失效；
#     2. USB 设备枚举正常但没有注册成 can 网卡
#        （设备被用户态程序卡住 / probe 时机错过），
#        表现为 can1 网卡不存在，只剩板载 can0；
#     3. can 网卡一直 DOWN，底盘反馈（0x211）永远收不到，
#        表现为：agent 启动显示 CAN 初始化成功但小车不动。
#
# 本脚本：
#   0. 检查 USB-CAN 设备是否已注册成 can 网卡；若没有，自动软恢复
#      （authorized 0/1 强制重枚举 + 重载 gs_usb 模块），仍不行则提示拔插；
#   A. 备份并重写损坏的 udev 规则（去掉多余右括号）；
#   B. reload udev 规则，重新触发设备/网卡；
#   C. 直接拉起所有 can 网卡（bitrate 500000, restart-ms 100）；
#   D. 对每个 can 网卡 candump 数秒，标出能收到底盘反馈（0x211）的口。
#
# 用法（需要 root）:
#   sudo bash fix_candle_usb.sh
# ============================================================
set -euo pipefail

RULE_FILE="/etc/udev/rules.d/99-gs_usb-candle.rules"
BITRATE="${1:-500000}"

# ---------------------------------------------------------------
# 0/5 检查 USB-CAN 设备是否已注册成 can 网卡
# ---------------------------------------------------------------
echo "== 0/5 检查 USB-CAN 是否已注册为 can 网卡 =="
if lsusb -d 1d50:606f >/dev/null 2>&1 || lsusb -d 1209:2323 >/dev/null 2>&1; then
    echo "  ✓ USB-CAN 设备已被系统枚举（lsusb 可见）"
    # 找出设备对应的 can 网卡（USB 总线上注册的）
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
    else
        echo "  ✗ 设备已枚举但【没有】注册成 can 网卡（gs_usb 驱动未绑定）。"
        echo "    尝试软恢复（authorized 0/1 强制重枚举 + 重载 gs_usb）..."
        # 找到设备 sysfs 路径（1d50:606f）
        DEVPATH=""
        for d in /sys/bus/usb/devices/*; do
            [[ -f "$d/idVendor" ]] || continue
            if grep -qi '^1d50$' "$d/idVendor" && grep -qi '^606f$' "$d/idProduct"; then
                DEVPATH="$d"
                break
            fi
        done
        if [[ -n "$DEVPATH" ]]; then
            name=$(basename "$DEVPATH")
            echo "    设备路径: $name"
            echo "    → 尝试 usbreset 软复位（recover_candle.sh 方案）..."
            BUS=$(lsusb -d 1d50:606f 2>/dev/null | awk '{print $2}')
            ADDR=$(lsusb -d 1d50:606f 2>/dev/null | awk '{print $4}' | tr -d ':')
            if [[ -n "$BUS" && -n "$ADDR" && -x /usr/bin/usbreset ]]; then
                /usr/bin/usbreset "/dev/bus/usb/$(printf '%03d' "$BUS")/$(printf '%03d' "$ADDR")" \
                    2>&1 || true
                sleep 3
            fi
            # 再次确认是否生成网卡
            for iface in /sys/class/net/can*; do
                [[ -e "$iface" ]] || continue
                dev=$(readlink -f "$iface/device" 2>/dev/null || true)
                if [[ "$dev" == *"usb"* ]]; then
                    CAN_USB=$(basename "$iface")
                    break
                fi
            done
            if [[ -n "$CAN_USB" ]]; then
                echo "    ✓ usbreset 软复位成功: $CAN_USB 已注册"
            else
                echo "    → 仍无网卡，尝试 authorized 0/1 强制重枚举..."
                echo 0 > "$DEVPATH/authorized" 2>/dev/null || echo "      (写 authorized=0 失败，可能无权限)"
                sleep 2
                echo 1 > "$DEVPATH/authorized" 2>/dev/null || true
                sleep 3
                # 再次确认
                for iface in /sys/class/net/can*; do
                    [[ -e "$iface" ]] || continue
                    dev=$(readlink -f "$iface/device" 2>/dev/null || true)
                    if [[ "$dev" == *"usb"* ]]; then
                        CAN_USB=$(basename "$iface")
                        break
                    fi
                done
                if [[ -n "$CAN_USB" ]]; then
                    echo "    ✓ authorized 重枚举成功: $CAN_USB 已注册"
                else
                    echo "    ✗ 仍无网卡。重载 gs_usb 模块再试..."
                    if modprobe -r gs_usb 2>/dev/null; then
                        sleep 1
                        modprobe gs_usb
                        sleep 3
                    else
                        echo "      (gs_usb 被占用，无法卸载；试试拔插适配器)"
                    fi
                    for iface in /sys/class/net/can*; do
                        [[ -e "$iface" ]] || continue
                        dev=$(readlink -f "$iface/device" 2>/dev/null || true)
                        if [[ "$dev" == *"usb"* ]]; then
                            CAN_USB=$(basename "$iface")
                            break
                        fi
                    done
                    if [[ -n "$CAN_USB" ]]; then
                        echo "    ✓ 重载模块后成功: $CAN_USB 已注册"
                    else
                        echo "    ✗ 仍无法注册为网卡！"
                        echo "      【最终手段】请【物理拔掉 USB-CAN 适配器，等 3 秒，再插回】,"
                        echo "      然后重新运行: sudo bash fix_candle_usb.sh"
                    fi
                fi
            fi
        fi
    fi
else
    echo "  ✗ USB-CAN 设备未被枚举！lsusb 看不到 1d50:606f。"
    echo "    请【物理拔掉 USB-CAN 适配器，等 3 秒，再插回】后重试。"
fi

echo
echo "== 1/5 检查 udev 规则 =="
if [[ -f "$RULE_FILE" ]]; then
    if grep -q '1d50.*606f' "$RULE_FILE"; then
        echo "  发现既有规则文件，先备份:"
        cp -a "$RULE_FILE" "$RULE_FILE.bak.$(date +%s)" && echo "  已备份"
    fi
fi

cat > "$RULE_FILE" <<'RULES'
SUBSYSTEM=="usb", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", MODE="0666", TAG+="uaccess"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="2323", MODE="0666", TAG+="uaccess"
ACTION=="add", SUBSYSTEM=="net", KERNEL=="can*", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", RUN+="/bin/sh -c '/sbin/ip link set %k down; /sbin/ip link set %k up type can bitrate 500000 restart-ms 100'"
ACTION=="add", SUBSYSTEM=="net", KERNEL=="can*", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="2323", RUN+="/bin/sh -c '/sbin/ip link set %k down; /sbin/ip link set %k up type can bitrate 500000 restart-ms 100'"
RULES
echo "  规则已重写（修复行尾多余右括号）:"
cat "$RULE_FILE"

echo
echo "== 2/5 重新加载 udev 规则 =="
udevadm control --reload-rules || true
udevadm trigger --subsystem-match=usb || true
sleep 1

echo
echo "== 3/5 拉起全部 can 网卡 =="
for iface in /sys/class/net/can*; do
    [[ -e "$iface" ]] || continue
    name=$(basename "$iface")
    echo "  can 网卡: $name"
    ip link set "$name" down 2>/dev/null || true
    ip link set "$name" up type can bitrate "$BITRATE" restart-ms 100 \
        && echo "    ↑ 已拉起 @ ${BITRATE}" \
        || echo "    ✗ 拉起失败（无 root 权限或驱动未加载？）"
done

echo
echo "== 4/5 探测底盘反馈（0x211）=="
FOUND=""
for iface in /sys/class/net/can*; do
    [[ -e "$iface" ]] || continue
    name=$(basename "$iface")
    echo "  --- $name ---"
    if timeout 2 candump -n 5 "$name" 2>/dev/null | grep -q '211'; then
        echo "    ✓ 收到底盘反馈帧 0x211 —— 底盘在这个口！"
        FOUND="$name"
    else
        # 有帧但没 0x211？还是完全无帧？再探一次给出更明确的结论
        if timeout 2 candump -n 1 "$name" >/dev/null 2>&1; then
            echo "    △ 收到 CAN 帧但未见 0x211（波特率不符/协议不同？）"
        else
            echo "    ✗ 2 秒内无任何 CAN 帧"
        fi
    fi
done

echo
if [[ -n "$FOUND" ]]; then
    echo "✅ 结论：底盘在 $FOUND 上。"
    echo "   启动 agent 时建议显式指定:"
    echo "     export BUNKER_CAN_CHANNEL=$FOUND && bash start_agent.sh"
    echo "   或快速验证:"
    echo "     python3 socketcan_chassis.py"
else
    echo "⚠ 未探测到任何底盘反馈。请检查："
    echo "  1. 底盘是否开机（指示灯/风扇）"
    echo "  2. CAN_H/CAN_L 是否接到 USB-CAN 适配器"
    echo "  3. lsusb 是否看到 1d50:606f（拔插一次）"
    echo "  4. 若仍无，可运行: bash reset_candle.sh 软复位适配器"
fi
