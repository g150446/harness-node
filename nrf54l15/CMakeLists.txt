# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# Find Zephyr
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Create project
project(voice_bridge_ble)

# Target sources
target_sources(app PRIVATE
    src/main.c
    src/adpcm.c
    src/audio_capture.c
)
