#!/usr/bin/env bash
# Bring up Jetson onboard can0 at Bunker Mini bitrate (500 kbps).
set -euo pipefail

BITRATE="${1:-500000}"

sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate "$BITRATE" restart-ms 100
ip -details -s link show can0
echo
echo "Listening 3s for chassis frames (expect 211# / 221#)..."
timeout 3 candump -td can0 || true
