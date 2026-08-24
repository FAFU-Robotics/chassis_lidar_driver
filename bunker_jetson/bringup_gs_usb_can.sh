#!/usr/bin/env bash
# Load kernel gs_usb and bring up the candleLight CAN iface at 500 kbps.
set -euo pipefail

BITRATE="${1:-500000}"

sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
sudo modprobe gs_usb

echo "Waiting for candleLight USB-CAN (1d50:606f)..."
for i in $(seq 1 20); do
  if lsusb -d 1d50:606f >/dev/null 2>&1 || lsusb -d 1209:2323 >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done

if ! lsusb -d 1d50:606f >/dev/null 2>&1 && ! lsusb -d 1209:2323 >/dev/null 2>&1; then
  echo "USB-CAN not found. Please plug candleLight, then re-run."
  exit 1
fi

# Wait for netdev (usually can1; can0 is Jetson mttcan)
IFACE=""
for i in $(seq 1 30); do
  for cand in /sys/class/net/can*; do
    [[ -e "$cand" ]] || continue
    name=$(basename "$cand")
    # skip onboard mttcan if distinguishable via device path
    if [[ -e "$cand/device/idVendor" ]] || readlink -f "$cand/device" | grep -qi usb; then
      IFACE=$name
      break 2
    fi
  done
  # fallback: any can* other than can0
  for name in can1 can2 can3; do
    if [[ -d "/sys/class/net/$name" ]]; then
      IFACE=$name
      break 2
    fi
  done
  sleep 0.2
done

# 设备在但 gs_usb 没绑定 → 尝试软恢复（usbreset / authorized 0/1）
if [[ -z "$IFACE" ]]; then
  echo "USB-CAN 枚举正常但没生成 can 网卡，尝试软恢复..."
  BUS=$(lsusb -d 1d50:606f 2>/dev/null | awk '{print $2}')
  ADDR=$(lsusb -d 1d50:606f 2>/dev/null | awk '{print $4}' | tr -d ':')
  if [[ -n "$BUS" && -n "$ADDR" && -x /usr/bin/usbreset ]]; then
    echo "  usbreset /dev/bus/usb/$(printf '%03d' "$BUS")/$(printf '%03d' "$ADDR") ..."
    usbreset "/dev/bus/usb/$(printf '%03d' "$BUS")/$(printf '%03d' "$ADDR")" >/dev/null 2>&1 || true
    sleep 3
  fi
  # 再找一次
  for i in $(seq 1 20); do
    for cand in /sys/class/net/can*; do
      [[ -e "$cand" ]] || continue
      name=$(basename "$cand")
      if [[ -e "$cand/device/idVendor" ]] || readlink -f "$cand/device" | grep -qi usb; then
        IFACE=$name
        break 2
      fi
    done
    sleep 0.3
  done
fi

if [[ -z "$IFACE" ]]; then
  echo "gs_usb loaded but no USB CAN netdev yet."
  echo "lsusb:"; lsusb | grep -iE '1d50|1209' || true
  echo "can ifaces:"; ip -br link | grep can || true
  echo "dmesg:"; journalctl -k -n 30 --no-pager | grep -iE 'gs_usb|can' || true
  echo
  echo "【最终手段】请【物理拔掉 USB-CAN 适配器，等 3 秒，再插回】，"
  echo "然后重新运行: sudo bash bringup_gs_usb_can.sh"
  echo "或运行一键恢复: sudo bash recover_can_now.sh"
  exit 2
fi

sudo ip link set "$IFACE" down 2>/dev/null || true
sudo ip link set "$IFACE" up type can bitrate "$BITRATE" restart-ms 100
ip -details -s link show "$IFACE"
echo
echo "OK: $IFACE @ ${BITRATE}. Test: candump $IFACE"
echo "Drive: python3 $(dirname "$0")/drive_bunker.py --interface socketcan --channel $IFACE --action forward"
echo "Avoid: python3 $(dirname "$0")/run_obstacle_avoidance.py --can-channel $IFACE"
