#!/bin/bash
# Build nrf54-motion, refresh the signed OTA payload, and USB-flash the board.
#
# Use this for initial provisioning or whenever the device needs to be updated
# over USB instead of BLE OTA.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-$HOME/nrf54-motion-build}"

"$SCRIPT_DIR/build_and_package_ota.sh" "$@"

PYOCD_CMD=""
for _p in "$HOME/.pyenv/shims/pyocd" "/usr/local/bin/pyocd"; do
    [ -x "$_p" ] && PYOCD_CMD="$_p" && break
done
if [ -z "$PYOCD_CMD" ]; then
    command -v pyocd >/dev/null 2>&1 && PYOCD_CMD="$(command -v pyocd)"
fi
if [ -z "$PYOCD_CMD" ]; then
    echo "ERROR: pyocd not found. Install with: pip install pyocd" >&2
    exit 1
fi

HEX_FILE="$BUILD_DIR/zephyr/zephyr.hex"
if [ ! -f "$HEX_FILE" ]; then
    HEX_FILE="$BUILD_DIR/merged.hex"
fi
if [ ! -f "$HEX_FILE" ]; then
    HEX_FILE="$BUILD_DIR/nrf54-motion/zephyr/zephyr.hex"
fi
if [ ! -f "$HEX_FILE" ]; then
    echo "ERROR: Expected hex not found under $BUILD_DIR" >&2
    exit 1
fi

echo "USB flashing $HEX_FILE with $PYOCD_CMD"
"$PYOCD_CMD" flash -t nrf54l "$HEX_FILE"

echo ""
echo "Done: built firmware, refreshed the OTA payload, and flashed over USB."
echo "Serial monitor at 115200 baud:"
echo "  screen \$(ls /dev/tty.usbmodem* | head -1) 115200"
echo "Expected log on motion: 'Motion detected! count=N delta=... step=... accel=(x, y, z) m/s^2'"
