#!/usr/bin/env bash
# Start the virtual drone for the DroneGuard live demo (run this ON THE PI).
#
#   ssh pi
#   ~/droneguard/pi/start_drone.sh               the laptop = the machine you ssh'd from
#   ~/droneguard/pi/start_drone.sh 192.168.1.5   or name the laptop's IP yourself
#
# No IP needs typing: over ssh, $SSH_CLIENT holds the laptop's current address,
# so this keeps working when the Wi-Fi network (and the laptop's IP) changes.
# Ctrl+C stops the drone.

set -e
cd "$(dirname "$0")/.."

HOST="${1:-${SSH_CLIENT%% *}}"
if [ -z "$HOST" ]; then
    echo "Which laptop runs DroneGuard? Run this over ssh from it, or: $0 <laptop-ip>"
    exit 1
fi
if [ ! -x .venv/bin/python ]; then
    echo "No Python environment in $(pwd)/.venv - see docs/pi_demo.md, 'Once: install on the Pi'"
    exit 1
fi

if pkill -f "live.drone --serve" 2>/dev/null; then
    echo "stopped the drone that was already running"
    sleep 0.5
fi

echo "Pi $(hostname -I | cut -d' ' -f1)  ->  DroneGuard on the laptop at $HOST (UDP 14550)"
echo "Now press 'Take off' on the laptop's live page. Ctrl+C here stops the drone."
exec .venv/bin/python -m live.drone --serve --host "$HOST"
