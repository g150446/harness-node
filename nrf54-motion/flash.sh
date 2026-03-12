#!/bin/bash
# Build and flash the nrf54-motion app for XIAO nRF54L15 Sense.
# Run this script from any directory; it locates the NCS toolchain
# automatically, falling back to a PATH-based west.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$SCRIPT_DIR"
PROJECT_DIR="/Users/g150446/projects/voice-harness/voice-bridge-ble"
BUILD_DIR="${BUILD_DIR:-$HOME/nrf54-motion-build}"
BOARD="xiao_nrf54l15/nrf54l15/cpuapp"
NCS_BASE="/opt/nordic/ncs/v2.9.2"
TOOLCHAIN_ROOT="/opt/nordic/ncs/toolchains"

# --- locate west --------------------------------------------------------
WEST=""
# 1) hard-coded NCS toolchain hash from repo docs
if [ -e "$TOOLCHAIN_ROOT/b8efef2ad5/bin/west" ] &&
   "$TOOLCHAIN_ROOT/b8efef2ad5/bin/python3" -m west --version >/dev/null 2>&1; then
    WEST="$TOOLCHAIN_ROOT/b8efef2ad5/bin/python3 -m west"
    export PATH="$TOOLCHAIN_ROOT/b8efef2ad5/bin:$PATH"
fi
# 2) any version under the toolchain root
if [ -z "$WEST" ] && [ -d "$TOOLCHAIN_ROOT" ]; then
    for _bin_dir in "$TOOLCHAIN_ROOT"/*/bin; do
        if [ -x "$_bin_dir/python3" ] &&
           "$_bin_dir/python3" -m west --version >/dev/null 2>&1; then
            WEST="$_bin_dir/python3 -m west"
            export PATH="$_bin_dir:$PATH"
            break
        fi
    done
fi
# 3) PATH
if [ -z "$WEST" ] && command -v west >/dev/null 2>&1; then
    WEST="$(command -v west)"
fi
if [ -z "$WEST" ]; then
    echo "ERROR: west not found. Install NCS v2.9.2 from https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/installation.html" >&2
    exit 1
fi
echo "west: $WEST"

# --- locate pyocd -------------------------------------------------------
PYOCD_CMD=""
for _p in "$HOME/.pyenv/shims/pyocd" "/usr/local/bin/pyocd"; do
    [ -x "$_p" ] && PYOCD_CMD="$_p" && break
done
command -v pyocd >/dev/null 2>&1 && PYOCD_CMD="$(command -v pyocd)"
if [ -z "$PYOCD_CMD" ]; then
    echo "ERROR: pyocd not found. Install with: pip install pyocd" >&2
    exit 1
fi

# --- build --------------------------------------------------------------
echo "Building $BOARD -> $BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$NCS_BASE"
eval "$WEST" build -p always -b "$BOARD" "$APP_DIR" \
    --build-dir "$BUILD_DIR" \
    -- -DBOARD_ROOT="$PROJECT_DIR"

# --- flash --------------------------------------------------------------
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
echo "Flashing $HEX_FILE with $PYOCD_CMD"
"$PYOCD_CMD" flash -t nrf54l "$HEX_FILE"

echo ""
echo "Flash complete. Open a serial monitor at 115200 baud:"
echo "  screen \$(ls /dev/tty.usbmodem* | head -1) 115200"
echo "Expected log on motion: 'Motion detected! count=N delta=... step=... accel=(x, y, z) m/s^2'"
