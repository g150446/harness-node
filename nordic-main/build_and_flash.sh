#!/bin/bash
# Build MCUboot + nordic-main and provision a XIAO nRF52840 Sense via UF2.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-$SCRIPT_DIR/build}"

BUILD_DIR="$BUILD_DIR" "$SCRIPT_DIR/build_and_package_ota.sh" "$@"

if [ -n "${NCS_BASE:-}" ] && [ -d "$NCS_BASE/zephyr" ]; then
    NCS_ROOT="$NCS_BASE"
else
    for candidate in \
        "/opt/nordic/ncs/2.9.2" \
        "/opt/nordic/ncs/v2.9.2" \
        "$HOME/ncs/v2.9.2" \
        "$HOME/ncs/2.9.2"; do
        if [ -d "$candidate/zephyr" ]; then
            NCS_ROOT="$candidate"
            break
        fi
    done
fi

if [ -z "${NCS_ROOT:-}" ]; then
    echo "ERROR: cannot locate the NCS workspace after the build." >&2
    exit 1
fi

MERGED_HEX="$BUILD_DIR/merged.hex"
MERGED_UF2="$BUILD_DIR/merged.uf2"
UF2CONV="$NCS_ROOT/zephyr/scripts/build/uf2conv.py"
PYTHON_CMD="${PYTHON3:-$(command -v python3 || true)}"

if [ ! -f "$MERGED_HEX" ]; then
    echo "ERROR: sysbuild merged image not found at $MERGED_HEX" >&2
    exit 1
fi
if [ ! -f "$UF2CONV" ]; then
    echo "ERROR: UF2 converter not found at $UF2CONV" >&2
    exit 1
fi
if [ -z "$PYTHON_CMD" ]; then
    echo "ERROR: python3 is required to generate UF2." >&2
    exit 1
fi

"$PYTHON_CMD" "$UF2CONV" -f 0xADA52840 -c -o "$MERGED_UF2" "$MERGED_HEX"
echo "Initial provisioning image: $MERGED_UF2 ($(wc -c < "$MERGED_UF2" | tr -d ' ') bytes)"
echo "Waiting for the XIAO UF2 drive. Double-tap the reset button now."

find_uf2_volume() {
    local volume
    for volume in "/Volumes/XIAO-SENSE" "/Volumes/XIAO BLE" "/Volumes/XIAO-BLE"; do
        if [ -d "$volume" ]; then
            printf '%s\n' "$volume"
            return
        fi
    done
    return 1
}

UF2_VOLUME=""
for second in $(seq 1 90); do
    if UF2_VOLUME="$(find_uf2_volume)"; then
        break
    fi
    if [ $((second % 10)) -eq 0 ]; then
        echo "  still waiting (${second}s)"
    fi
    sleep 1
done

if [ -z "$UF2_VOLUME" ]; then
    echo "ERROR: UF2 drive did not appear within 90 seconds." >&2
    exit 1
fi

echo "UF2 drive found at $UF2_VOLUME"
cp "$MERGED_UF2" "$UF2_VOLUME/"
sync
echo "UF2 copy completed; waiting for the application USB serial port."

for second in $(seq 1 45); do
    serial_port="$(find /dev -maxdepth 1 -name 'cu.usbmodem*' -print -quit 2>/dev/null || true)"
    if [ -n "$serial_port" ] && [ ! -d "$UF2_VOLUME" ]; then
        echo "Provisioning complete. Application serial port: $serial_port"
        exit 0
    fi
    sleep 1
done

echo "WARNING: UF2 was copied, but application USB re-enumeration was not confirmed." >&2
exit 1
