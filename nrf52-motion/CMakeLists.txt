# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Tell Partition Manager to place the app at code_partition (0x27000),
# not at 0x0, so the linker uses the correct VMA for the Adafruit bootloader.
set(PM_STATIC_YML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/pm_static.yml)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nrf52_motion)

target_sources(app PRIVATE src/main.c)
