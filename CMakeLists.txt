cmake_minimum_required(VERSION 3.16)

# Board selection: stickc_plus2 (default) | atom_echo_s3r
#   idf.py -DHN_BOARD=stickc_plus2 set-target esp32 build
#   idf.py -DHN_BOARD=atom_echo_s3r set-target esp32s3 build
if(NOT DEFINED HN_BOARD)
    if(DEFINED ENV{HN_BOARD})
        set(HN_BOARD $ENV{HN_BOARD})
    else()
        set(HN_BOARD "stickc_plus2")
    endif()
endif()

if(HN_BOARD STREQUAL "atom_echo_s3r")
    set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/atom_echo_s3r")
elseif(HN_BOARD STREQUAL "stickc_plus2")
    set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/stickc_plus2")
else()
    message(FATAL_ERROR "Unknown HN_BOARD='${HN_BOARD}'. Use stickc_plus2 or atom_echo_s3r")
endif()

message(STATUS "HarnessNode ESP board component: ${HN_BOARD} -> ${EXTRA_COMPONENT_DIRS}")

# App version embedded in esp_app_desc (SMP OTA / img state).
if(HN_BOARD STREQUAL "stickc_plus2")
    file(READ "${CMAKE_CURRENT_LIST_DIR}/stickc_plus2/VERSION" _hn_ver)
    string(STRIP "${_hn_ver}" PROJECT_VER)
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(voice_bridge_ble)
