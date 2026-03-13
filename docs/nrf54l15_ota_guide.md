# nRF54L15 OTA Guide

## Overview

This document describes the BLE OTA implementation used by `nrf54-motion/` on the Seeed Studio XIAO nRF54L15 Sense.

The current OTA path uses:

- MCUboot
- Zephyr sysbuild
- MCUmgr image management
- SMP over BLE
- a Python updater at `mac_client/ota_updater.py`

This is the OTA path that was verified on hardware. It is separate from the `nrf54l15/` audio firmware.

## Scope

- **OTA-enabled app:** `nrf54-motion/`
- **Target board:** `xiao_nrf54l15/nrf54l15/cpuapp`
- **Transport:** BLE SMP
- **Boot mode:** MCUboot swap without scratch
- **Upgrade style in current flow:** upload to secondary slot, mark test image, reboot, let MCUboot swap

## File map

```text
nrf54-motion/
├── prj.conf
├── sysbuild.conf
├── pm_static.yml
├── flash.sh
├── ota_update.bin
└── sysbuild/mcuboot/
    ├── prj.conf
    └── boards/xiao_nrf54l15_nrf54l15_cpuapp.conf

mac_client/
└── ota_updater.py
```

## BLE-facing OTA behavior

The device advertises as `MotionBridge`.

Relevant GATT UUIDs:

- Motion Service: `00000010-0000-1000-8000-00805f9b34fb`
- Motion Notify: `00000011-0000-1000-8000-00805f9b34fb`
- Build Info: `00000012-0000-1000-8000-00805f9b34fb`
- SMP Service: `8D53DC1D-1DB7-4CD3-868B-8A527460AA84`
- SMP Characteristic: `DA2E7828-FBCE-4E01-AE9E-261174997C48`

The Build Info characteristic returns `__DATE__ " " __TIME__` from the running firmware image. This is the easiest way to confirm that the device is actually running the post-OTA image.

## Build artifacts

`nrf54-motion/flash.sh` is the recommended entrypoint.

It performs all of the following:

1. Builds with sysbuild and MCUboot enabled
2. Flashes `merged.hex` over CMSIS-DAP / `pyocd`
3. Copies the signed OTA payload to `nrf54-motion/ota_update.bin`

Important artifacts:

- USB flash image: `build/merged.hex` or `${BUILD_DIR}/merged.hex`
- OTA payload: `${BUILD_DIR}/nrf54-motion/zephyr/zephyr.signed.bin`
- Repo-local OTA payload copy: `nrf54-motion/ota_update.bin`

## Recommended usage

### 1. Prepare host tools

```bash
cd mac_client
python3 -m venv ../venv  # if needed
../venv/bin/pip install -r requirements.txt
../venv/bin/pip install cbor2
```

`cbor2` is required by `ota_updater.py`.

### 2. Flash a known baseline over USB

```bash
cd nrf54-motion
./flash.sh
```

This ensures MCUboot and the baseline application image are in a known-good state before trying OTA.

### 3. Create the OTA image

Make your firmware change, then rebuild so that `nrf54-motion/ota_update.bin` reflects the new image.

If you want to confirm the image change with Build Info only, make sure the build timestamp will differ from the running firmware.

### 4. Run the BLE OTA updater

```bash
cd mac_client
../venv/bin/python3 ota_updater.py ../nrf54-motion/ota_update.bin
```

Expected flow:

1. scan for `MotionBridge`
2. connect over BLE
3. negotiate MTU
4. upload the signed image to slot 1
5. query image state
6. set the image test flag
7. reset the device
8. let MCUboot swap on next boot

### 5. Verify the new image

Read the Build Info characteristic before and after OTA. A changed timestamp indicates the board rebooted into the new image.

## Configuration requirements

### Application-side requirements

The application-side OTA path depends on these areas in `nrf54-motion/prj.conf`:

- MCUmgr / SMP enabled
- image management enabled
- flash map enabled
- settings enabled
- BLE MTU increased for transfer throughput

Most importantly:

```kconfig
CONFIG_NRF_RRAM_READYNEXT_TIMEOUT_VALUE=0
```

This is required on XIAO nRF54L15 so the running application can successfully write MCUboot trailer metadata when handling `IMG_STATE` writes.

### MCUboot-side requirements

`nrf54-motion/sysbuild/mcuboot/boards/xiao_nrf54l15_nrf54l15_cpuapp.conf` contains board-specific MCUboot workarounds for this target, including:

- `CONFIG_FPROTECT=n`
- `CONFIG_FLASH=y`
- `CONFIG_SOC_FLASH_NRF_RRAM=y`
- `CONFIG_NRF_RRAM_READYNEXT_TIMEOUT_VALUE=0`
- `CONFIG_NRF_GRTC_TIMER=n`
- `CONFIG_SYS_CLOCK_EXISTS=n`

Do not remove these casually; they were needed to keep MCUboot stable on this board/NCS combination.

## Partitioning notes

Static partitioning is defined in `nrf54-motion/pm_static.yml`.

Key regions:

- MCUboot at `0x000000`
- primary image span starting at `0x00F800`
- secondary image slot at `0x0B2000`
- settings storage at `0x15C000`

The current OTA implementation assumes:

- one updateable image
- a valid secondary slot
- MCUboot swap-without-scratch mode

If you change the slot sizes or layout, re-check trailer placement and signed image size.

## Important cautions

### 1. `nrf54-motion` and `nrf54l15` are different apps

The verified OTA flow in this repository is for `nrf54-motion/`. Do not assume that the same instructions apply to `nrf54l15/` without additional work.

### 2. The updater performs a test boot request

`ota_updater.py` uploads the image, sends `IMG_STATE` with `confirm=false`, and resets the board.

That means:

- the requested boot is a **test boot**
- MCUboot decides the slot swap on reboot
- the updater does **not** currently reconnect and confirm the post-boot image automatically

### 3. Future confirmation / rollback work may still be needed

The verified flow confirms:

- upload works
- pending/test flag write works
- reboot into the new image works

If you later need explicit image confirmation or automated rollback testing, add that logic deliberately and verify it on hardware.

### 4. `cbor2` is easy to forget

`mac_client/requirements.txt` does not currently provide `cbor2`, so OTA users must install it separately unless that dependency list is updated later.

### 5. Hardware verification matters

For this target, build success alone is not enough. OTA-related changes should be validated on a real board because several past failures were board- and RRAM-specific.

## Maintenance checklist

When touching OTA-related code, verify at least these points:

1. `flash.sh` still produces a signed OTA image
2. the app still advertises as `MotionBridge`
3. the SMP characteristic is present
4. upload reaches 100%
5. `Image test flag set.` appears
6. the board reboots
7. Build Info changes to the new image

## Related documents

- `docs/nrf54l15_ota_status.md`
- `README.md`
