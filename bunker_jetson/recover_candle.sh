#!/usr/bin/env bash
# Recover wedged candleLight without physical unplug (uses usbreset).
# If this fails, unplug USB-CAN for 3 seconds and plug back in.
set -euo pipefail

if ! lsusb -d 1d50:606f >/dev/null; then
  echo "candleLight not found. Please plug it in."
  exit 1
fi

BUS=$(lsusb -d 1d50:606f | awk '{print $2}')
ADDR=$(lsusb -d 1d50:606f | awk '{print $4}' | tr -d ':')
DEVNODE=$(printf '/dev/bus/usb/%03d/%03d' "$BUS" "$ADDR")
echo "Resetting $DEVNODE ..."
sudo usbreset "$DEVNODE"
sleep 2
lsusb -d 1d50:606f
echo "Done. Next: bash $(dirname "$0")/bringup_gs_usb_can.sh 然后  python3 $(dirname "$0")/socketcan_chassis.py"
