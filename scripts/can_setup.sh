#!/bin/bash
# 配置并启用 SocketCAN 接口 (需 sudo). 用法: ./can_setup.sh [iface=can0]
set -euo pipefail
IFACE="${1:-can0}"
sudo ip link set "$IFACE" type can bitrate 500000
sudo ip link set "$IFACE" up
echo "[can_setup] $IFACE up @500k:"
ip -details link show "$IFACE" | grep -i bitrate || true
