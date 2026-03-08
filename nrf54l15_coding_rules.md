# AI Coding Agent Instructions: nRF54L15 Development

## 1. Project Context

**Target Chip:** Nordic Semiconductor nRF54L15 (Sense)

**SDK:** nRF Connect SDK (NCS) v2.9.2

**Target Board:** `nrf54l15dk/nrf54l15/cpuapp` (or specify your custom board)

**Toolchain Path:** `/opt/nordic/ncs/toolchains/b8efef2ad5/bin`

---

## 2. Toolchain Restrictions (IMPORTANT)

⚠️ **Do NOT use `nrfjprog`**. It is legacy and does not fully support the nRF54L series' lifecycle management and memory architecture.

✅ **Use `west`** for building and flashing.

✅ **Use `nrfutil device`** for hardware inspection and advanced programming.

---

## 3. Preferred Commands

### Building

Always use `west build`.

```bash
# Initial build
west build -b nrf54l15dk/nrf54l15/cpuapp

# Rebuild after changes
west build
```

### Flashing

Use `west flash`. This command internally calls `nrfutil-device`, which is compatible with nRF54L15.

```bash
west flash
```

### Device Inspection

To list connected devices or find serial numbers, use `nrfutil`.

```bash
nrfutil device list
```

---

## 4. Coding & Architecture Guidelines

### Lifecycle Management

⚠️ **Be aware that nRF54L uses LCS (Life Cycle State).** If a programming error occurs, check the device status via:

```bash
nrfutil device x-provisions-view
```

### Devicetree

- Use standard Zephyr Devicetree (DTS) patterns
- For nRF54L15, ensure hardware peripherals are correctly assigned to the **cpuapp** (Application Core)

### Logging

- Use `CONFIG_LOG=y`
- Use RTT or UART for debugging

---

## 5. Instructions for the Agent

When I ask to **"build and flash"**, execute the following sequence:

1. **Validate** the Devicetree and Kconfig
2. **Run** `west build -b nrf54l15dk/nrf54l15/cpuapp`
3. **If successful**, run `west flash`
4. **If a "legacy tool" error or nrfjprog missing error occurs:**
   - Do NOT attempt to install nrfjprog
   - Verify that the environment is using `nrfutil` as the runner

---

## 6. Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| `nrfjprog not found` | Use `west flash` (uses nrfutil internally) |
| `Board not found` | Check board name: `nrf54l15dk/nrf54l15/cpuapp` |
| `LCS error` | Check device lifecycle with `nrfutil device x-provisions-view` |
| `Kconfig warnings` | Remove undefined CONFIG_* options from prj.conf |

### Flash Script

Use the provided `flash.sh` script for automated build and flash:

```bash
cd nrf54l15
./flash.sh
```

---

## 7. File Structure

```
nrf54l15/
├── CMakeLists.txt          # Build configuration
├── prj.conf                # Kconfig options
├── west.yml                # Zephyr manifest
├── flash.sh                # Automated flash script
├── src/
│   ├── main.c              # Main application
│   ├── adpcm.c/h           # ADPCM codec
│   └── audio_capture.c/h   # Audio capture module
└── boards/
    └── xiao_nrf54l15_sense.overlay  # Board-specific DTS overlay
```

---

## 8. References

- [nRF54L15 DK Documentation](https://docs.nordicsemi.com/bundle/ug_nrf54l15_dk/page/UG/nrf54l15dk/introduction.html)
- [Zephyr Project Documentation](https://docs.zephyrproject.org/)
- [nRF Connect SDK Documentation](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/introduction.html)
- [nrfutil Device Documentation](https://docs.nordicsemi.com/bundle/ug_nrfutil_device/page/UG/nrfutil_device/introduction.html)
