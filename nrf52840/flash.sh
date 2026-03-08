#!/bin/bash
#
# Voice Bridge BLE - XIAO nRF52840 Sense Flash Script (UF2 Bootloader)
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

BOARD="xiao_ble/nrf52840/sense"
BUILD_DIR="$NCS_BASE/build"
SAMPLE_DIR="$NCS_BASE/nrf/samples/voice_bridge"

echo "============================================"
echo "Voice Bridge BLE - XIAO nRF52840 Flasher"
echo "(UF2 Bootloader Mode)"
echo "============================================"
echo ""

# Step 1: Copy source files
echo -e "${YELLOW}Step 1: Copying source files...${NC}"

if [ ! -d "$SAMPLE_DIR" ]; then
    echo "Creating sample directory from peripheral_lbs..."
    cp -r "$NCS_BASE/nrf/samples/bluetooth/peripheral_lbs" "$SAMPLE_DIR"
fi

echo "Copying main.c..."
cp "$PROJECT_DIR/nrf52840/src/main.c" "$SAMPLE_DIR/src/"
echo "Copying adpcm.c/h..."
cp "$PROJECT_DIR/nrf52840/src/adpcm.c" "$SAMPLE_DIR/src/"
cp "$PROJECT_DIR/nrf52840/src/adpcm.h" "$SAMPLE_DIR/src/"
echo "Copying CMakeLists.txt..."
cp "$PROJECT_DIR/nrf52840/CMakeLists.txt" "$SAMPLE_DIR/"

echo "Creating prj.conf..."
cat > "$SAMPLE_DIR/prj.conf" << 'EOF'
CONFIG_MAIN_STACK_SIZE=4096
CONFIG_HEAP_MEM_POOL_SIZE=49152
CONFIG_LOG=y
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y
CONFIG_BT=y
CONFIG_BT_PERIPHERAL=y
CONFIG_BT_BROADCASTER=y
CONFIG_BT_MAX_CONN=2
CONFIG_BT_SMP=n
CONFIG_GPIO=y
CONFIG_GPIO_NRFX=y
CONFIG_SYS_CLOCK_TICKS_PER_SEC=32768
CONFIG_PRINTK=y
EOF

echo ""

# Step 2: Clean and build
echo -e "${YELLOW}Step 2: Cleaning build directory...${NC}"
rm -rf "$BUILD_DIR"

echo -e "${YELLOW}Step 3: Building firmware...${NC}"
echo "This may take a few minutes..."
echo ""

cd "$NCS_BASE"
if west build -b "$BOARD" nrf/samples/voice_bridge; then
    echo -e "${GREEN}Build successful!${NC}"
else
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi

echo ""

# Step 4: Flash using UF2 bootloader
echo -e "${YELLOW}Step 4: Flashing to XIAO nRF52840...${NC}"
echo ""
echo "Please put XIAO nRF52840 in bootloader mode:"
echo "  1. Press and hold the RESET button"
echo "  2. While holding RESET, press BOOT (or press RESET twice quickly)"
echo "  3. Release RESET"
echo "  4. A drive named 'XIAO-NRF52' should appear"
echo ""
echo "Waiting for bootloader mode..."

# Wait for UF2 drive to appear
MAX_WAIT=60
WAIT_COUNT=0
UF2_DRIVE=""

while [ $WAIT_COUNT -lt $MAX_WAIT ]; do
    # Check for XIAO-NRF52 drive
    if [ -d "/Volumes/XIAO-NRF52" ]; then
        UF2_DRIVE="/Volumes/XIAO-NRF52"
        break
    fi
    if [ -d "/Volumes/XIAO" ]; then
        UF2_DRIVE="/Volumes/XIAO"
        break
    fi
    if [ -d "/Volumes/XIAO_NRF52" ]; then
        UF2_DRIVE="/Volumes/XIAO_NRF52"
        break
    fi
    
    sleep 1
    WAIT_COUNT=$((WAIT_COUNT + 1))
    if [ $((WAIT_COUNT % 5)) -eq 0 ]; then
        echo "  Waiting... ($WAIT_COUNT seconds)"
    fi
done

if [ -z "$UF2_DRIVE" ]; then
    echo -e "${RED}Timeout: Bootloader mode not detected.${NC}"
    echo ""
    echo "Please try again:"
    echo "  1. Disconnect USB cable"
    echo "  2. Press and hold RESET button"
    echo "  3. While holding RESET, reconnect USB cable"
    echo "  4. Wait 1 second, then release RESET"
    echo "  5. Run this script again"
    exit 1
fi

echo ""
echo -e "${GREEN}Bootloader detected at: $UF2_DRIVE${NC}"
echo "Copying firmware..."

# Copy UF2 file
UF2_FILE="$BUILD_DIR/voice_bridge/zephyr/zephyr.uf2"
if [ -f "$UF2_FILE" ]; then
    cp "$UF2_FILE" "$UF2_DRIVE/"
    echo -e "${GREEN}Firmware copied successfully!${NC}"
    echo ""
    echo "The device will automatically reset and start the new firmware."
    echo "Wait a few seconds for the device to boot..."
    sleep 5
else
    echo -e "${RED}Error: UF2 file not found at $UF2_FILE${NC}"
    exit 1
fi

# Eject the drive
echo "Ejecting drive..."
diskutil eject "$UF2_DRIVE" 2>/dev/null || true

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
