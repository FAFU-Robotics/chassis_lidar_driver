#!/usr/bin/env bash
# Soft-recover candleLight USB-CAN. If this fails, physically unplug/replug.
set -euo pipefail

echo "Looking for candleLight (1d50:606f)..."
PATHS=()
for d in /sys/bus/usb/devices/*; do
  [[ -f "$d/idVendor" ]] || continue
  if grep -qi '^1d50$' "$d/idVendor" && grep -qi '^606f$' "$d/idProduct"; then
    PATHS+=("$d")
  fi
done

if [[ ${#PATHS[@]} -eq 0 ]]; then
  echo "Not found. Please physically plug in the USB-CAN, then re-run."
  exit 1
fi

for d in "${PATHS[@]}"; do
  name=$(basename "$d")
  echo "Found $name ($(cat "$d/product" 2>/dev/null || echo candle))"
  echo "  unbind/bind..."
  echo "$name" | sudo tee /sys/bus/usb/drivers/usb/unbind >/dev/null || true
  sleep 1
  echo "$name" | sudo tee /sys/bus/usb/drivers/usb/bind >/dev/null || true
  sleep 1
  if [[ -f "$d/authorized" ]]; then
    echo "  authorized 0/1..."
    echo 0 | sudo tee "$d/authorized" >/dev/null || true
    sleep 1
    echo 1 | sudo tee "$d/authorized" >/dev/null || true
    sleep 1
  fi
done

sleep 1
if lsusb | grep -q '1d50:606f'; then
  echo "lsusb OK:"
  lsusb | grep '1d50:606f'
  echo
  echo "Next:"
  echo "  bash $(dirname "$0")/bringup_gs_usb_can.sh"
  echo "  python3 $(dirname "$0")/socketcan_chassis.py   # 打印 CAN 网卡状态与 USB 归属"
else
  echo "Still missing. Please PHYSICAL unplug USB-CAN for 3s, plug back, then:"
  echo "  lsusb | grep 1d50"
  exit 2
fi
