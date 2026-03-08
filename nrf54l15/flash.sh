#!/bin/bash
#
# Voice Bridge BLE - XIAO nRF54L15 Sense Flash Script
#

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

NCS_TOOLCHAIN_DIR="/opt/nordic/ncs/toolchains"
if [ -d "$NCS_TOOLCHAIN_DIR" ]; then
    NCS_VERSION=$(ls -1 "$NCS_TOOLCHAIN_DIR" | head -1)
    NCS_PATH="$NCS_TOOLCHAIN_DIR/$NCS_VERSION"
    NCS_BASE="/opt/nordic/ncs/v2.9.2"
else
    echo -e "${RED}Error: NCS toolchain not found${NC}"
    exit 1
fi

export PATH="$NCS_PATH/bin:$PATH"

BOARD="nrf54l15dk/nrf54l15/cpuapp"
BUILD_DIR="$NCS_BASE/build"
SAMPLE_DIR="$NCS_BASE/nrf/samples/voice_bridge_nrf54l15"

echo "============================================"
echo "Voice Bridge BLE - XIAO nRF54L15 Flasher"
echo "============================================"
echo ""
echo "NCS Path: $NCS_PATH"
echo "NCS Base: $NCS_BASE"
echo "Board: $BOARD"
echo ""

# Step 1: Copy source files
echo -e "${YELLOW}Step 1: Copying source files...${NC}"

if [ ! -d "$SAMPLE_DIR" ]; then
    echo "Creating sample directory..."
    mkdir -p "$SAMPLE_DIR/src"
fi

echo "Copying main.c..."
cp "$PROJECT_DIR/nrf54l15/src/main.c" "$SAMPLE_DIR/src/"
echo "Copying adpcm.c/h..."
cp "$PROJECT_DIR/nrf54l15/src/adpcm.c" "$SAMPLE_DIR/src/"
cp "$PROJECT_DIR/nrf54l15/src/adpcm.h" "$SAMPLE_DIR/src/"
echo "Copying audio_capture.c/h..."
cp "$PROJECT_DIR/nrf54l15/src/audio_capture.c" "$SAMPLE_DIR/src/"
cp "$PROJECT_DIR/nrf54l15/src/audio_capture.h" "$SAMPLE_DIR/src/"
echo "Copying CMakeLists.txt..."
cp "$PROJECT_DIR/nrf54l15/CMakeLists.txt" "$SAMPLE_DIR/"

echo "Creating prj.conf..."
cp "$PROJECT_DIR/nrf54l15/prj.conf" "$SAMPLE_DIR/prj.conf"

echo "Copying board overlay..."
mkdir -p "$SAMPLE_DIR/boards"
cp "$PROJECT_DIR/nrf54l15/boards/"*.overlay "$SAMPLE_DIR/boards/" 2>/dev/null || true

echo ""

# Step 2: Clean and build
echo -e "${YELLOW}Step 2: Cleaning build directory...${NC}"
rm -rf "$BUILD_DIR"

echo -e "${YELLOW}Step 3: Building firmware...${NC}"
echo "This may take a few minutes..."
echo ""

cd "$NCS_BASE"
if west build -b "$BOARD" nrf/samples/voice_bridge_nrf54l15; then
    echo -e "${GREEN}Build successful!${NC}"
else
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi

echo ""

# Step 4: Flash
echo -e "${YELLOW}Step 4: Flashing to XIAO nRF54L15...${NC}"
echo ""
echo "Flashing..."

# XIAO nRF54L15 uses CMSIS-DAP, so flash via pyocd
HEX_FILE="$BUILD_DIR/merged.hex"
if [ ! -f "$HEX_FILE" ]; then
    HEX_FILE="$BUILD_DIR/voice_bridge_nrf54l15/zephyr/zephyr.hex"
fi

echo "Hex file: $HEX_FILE"

# Use system pyocd (not NCS toolchain python) to ensure nrf54l target is available
PYOCD_CMD="$(command -v pyocd 2>/dev/null || echo "$HOME/.pyenv/shims/pyocd")"
"$PYOCD_CMD" flash -t nrf54l "$HEX_FILE" || {
    echo -e "${RED}pyocd flash failed. Ensure the device is connected and pyocd is installed.${NC}"
    echo "  Install: pip install pyocd"
    exit 1
}

echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}Flash complete!${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo "Next steps:"
echo "  1. Wait a few seconds for the device to boot"
echo "  2. Run Mac client: cd $PROJECT_DIR/mac_client && python3 voice_bridge_recorder.py"
echo "  3. Press 'r' to start recording, 's' to stop"
echo ""
