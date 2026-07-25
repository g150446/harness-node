#!/bin/bash
# Build nordic-main with MCUboot and refresh the signed BLE OTA payload.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$SCRIPT_DIR"
BUILD_DIR="${BUILD_DIR:-$APP_DIR/build}"
BOARD="${BOARD:-xiao_ble/nrf52840/sense}"
NCS_VERSION="${NCS_VERSION:-v2.9.2}"

find_ncs_base() {
    if [ -n "${NCS_BASE:-}" ] && [ -d "$NCS_BASE/zephyr" ]; then
        printf '%s\n' "$NCS_BASE"
        return
    fi

    local candidate
    for candidate in \
        "/opt/nordic/ncs/2.9.2" \
        "/opt/nordic/ncs/v2.9.2" \
        "$HOME/ncs/v2.9.2" \
        "$HOME/ncs/2.9.2"; do
        if [ -d "$candidate/zephyr" ]; then
            printf '%s\n' "$candidate"
            return
        fi
    done

    return 1
}

if ! NCS_ROOT="$(find_ncs_base)"; then
    echo "ERROR: nRF Connect SDK v2.9.2 was not found." >&2
    echo "Set NCS_BASE to the SDK workspace containing zephyr/, or install it with nrfutil sdk-manager." >&2
    exit 1
fi

WEST_CMD=()
if [ -n "${WEST:-}" ] && [ -x "$WEST" ]; then
    WEST_CMD=("$WEST")
elif command -v west >/dev/null 2>&1; then
    WEST_CMD=("$(command -v west)")
elif [ -n "${NRFUTIL:-}" ] && [ -x "$NRFUTIL" ]; then
    WEST_CMD=("$NRFUTIL" sdk-manager toolchain launch --ncs-version "$NCS_VERSION" -- west)
elif command -v nrfutil >/dev/null 2>&1; then
    WEST_CMD=("$(command -v nrfutil)" sdk-manager toolchain launch --ncs-version "$NCS_VERSION" -- west)
else
    echo "ERROR: neither west nor nrfutil is available." >&2
    exit 1
fi

echo "NCS: $NCS_ROOT"
echo "Board: $BOARD"
echo "Build directory: $BUILD_DIR"

# Pin nested west invocations (including Zephyr post-build tools) to this NCS.
export WEST_CONFIG_LOCAL="$NCS_ROOT/.west/config"

mkdir -p "$BUILD_DIR"
cd "$NCS_ROOT"
"${WEST_CMD[@]}" build -p always --sysbuild -b "$BOARD" "$APP_DIR" \
    --build-dir "$BUILD_DIR"

OTA_BIN="$BUILD_DIR/nordic-main/zephyr/zephyr.signed.bin"
if [ ! -f "$OTA_BIN" ]; then
    echo "ERROR: signed OTA image was not generated at $OTA_BIN" >&2
    exit 1
fi

OTA_SIZE="$(wc -c < "$OTA_BIN" | tr -d ' ')"
SLOT_SIZE=$((0x52000))
if [ "$OTA_SIZE" -gt "$SLOT_SIZE" ]; then
    echo "ERROR: signed image ($OTA_SIZE bytes) exceeds the OTA slot ($SLOT_SIZE bytes)." >&2
    exit 1
fi

cp "$OTA_BIN" "$APP_DIR/ota_update.bin"
echo "OTA payload: $APP_DIR/ota_update.bin ($OTA_SIZE bytes)"
echo "Done. No device was flashed."
