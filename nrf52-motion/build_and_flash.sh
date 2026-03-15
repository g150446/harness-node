#!/bin/bash
# Build nrf52-motion with MCUboot OTA support and flash over USB (UF2).
#
# Initial flash: double-tap reset → XIAO-SENSE drive → script copies merged UF2
# OTA updates: use build_and_package_ota.sh + mac_client/ota_updater.py

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$SCRIPT_DIR"
PROJECT_DIR="/Users/g150446/projects/voice-harness/voice-bridge-ble"
BUILD_DIR="${BUILD_DIR:-$HOME/nrf52-motion-build}"
BOARD="xiao_ble/nrf52840/sense"
NCS_BASE="/opt/nordic/ncs/v2.9.2"
TOOLCHAIN_ROOT="/opt/nordic/ncs/toolchains"

# Locate west
WEST=""
if [ -e "$TOOLCHAIN_ROOT/b8efef2ad5/bin/west" ] &&
   "$TOOLCHAIN_ROOT/b8efef2ad5/bin/python3" -m west --version >/dev/null 2>&1; then
    WEST="$TOOLCHAIN_ROOT/b8efef2ad5/bin/python3 -m west"
    export PATH="$TOOLCHAIN_ROOT/b8efef2ad5/bin:$PATH"
fi
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
if [ -z "$WEST" ] && command -v west >/dev/null 2>&1; then
    WEST="$(command -v west)"
fi
if [ -z "$WEST" ]; then
    echo "ERROR: west not found." >&2
    exit 1
fi
echo "west: $WEST"

# Build with sysbuild (MCUboot + app)
echo "Building $BOARD -> $BUILD_DIR (sysbuild with MCUboot)"
mkdir -p "$BUILD_DIR"
cd "$NCS_BASE"
eval "$WEST" build -p always --sysbuild -b "$BOARD" "$APP_DIR" \
    --build-dir "$BUILD_DIR" \
    -- -DBOARD_ROOT="$PROJECT_DIR"

# Copy signed OTA payload
OTA_BIN="$BUILD_DIR/nrf52-motion/zephyr/zephyr.signed.bin"
if [ -f "$OTA_BIN" ]; then
    cp "$OTA_BIN" "$APP_DIR/ota_update.bin"
    echo "OTA payload: $APP_DIR/ota_update.bin ($(wc -c < "$APP_DIR/ota_update.bin") bytes)"
fi

# Generate merged UF2 (MCUboot + signed app) for initial flash
MERGED_HEX="$BUILD_DIR/merged.hex"
MERGED_UF2="$BUILD_DIR/merged.uf2"
UF2CONV="$NCS_BASE/zephyr/scripts/build/uf2conv.py"
PYTHON3="$TOOLCHAIN_ROOT/b8efef2ad5/bin/python3"

if [ -f "$MERGED_HEX" ] && [ -f "$UF2CONV" ]; then
    echo "Converting merged.hex to UF2..."
    "$PYTHON3" "$UF2CONV" -f 0xADA52840 -c -o "$MERGED_UF2" "$MERGED_HEX"
    echo "Merged UF2: $MERGED_UF2 ($(wc -c < "$MERGED_UF2") bytes)"
elif [ ! -f "$MERGED_HEX" ]; then
    echo "WARNING: merged.hex not found at $MERGED_HEX" >&2
fi

# Flash via UF2
if [ -f "$MERGED_UF2" ]; then
    echo ""
    echo "Waiting for XIAO-SENSE UF2 drive (double-tap reset button)..."
    for _i in $(seq 1 60); do
        sleep 1
        if [ -d /Volumes/XIAO-SENSE ]; then
            echo "Found XIAO-SENSE! Flashing merged UF2 (MCUboot + app)..."
            rsync --no-perms --no-owner --no-group "$MERGED_UF2" /Volumes/XIAO-SENSE/ 2>&1 || true
            echo ""
            echo "Done. After reboot: MCUboot at 0x27000 → app at 0x33200"
            echo "Serial monitor (USB CDC ACM):"
            echo "  screen \$(ls /dev/tty.usbmodem* | head -1) 115200"
            echo "OTA update: python3 mac_client/ota_updater.py nrf52-motion/ota_update.bin"
            exit 0
        fi
        _serial=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)
        if [ -n "$_serial" ]; then
            echo "  Device running on $_serial - double-tap reset button to enter bootloader"
        fi
        [ $((_i % 10)) -eq 0 ] && echo "  ${_i}s..."
    done
    echo "Timeout. Flash manually:"
    echo "  rsync --no-perms --no-owner --no-group $MERGED_UF2 /Volumes/XIAO-SENSE/"
fi
