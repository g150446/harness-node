cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/atom_echo_s3r")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(voice_bridge_ble)
