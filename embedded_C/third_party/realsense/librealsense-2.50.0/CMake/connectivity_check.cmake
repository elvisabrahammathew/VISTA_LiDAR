# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.

# VISTA disables the two SDK features that download firmware. Avoid an
# unnecessary network request during every offline/cross-platform configure.
if(NOT BUILD_WITH_TM2 AND NOT IMPORT_DEPTH_CAM_FW)
    message(STATUS "Skipping librealsense connectivity check (downloads disabled)")
    set(INTERNET_CONNECTION OFF)
    return()
endif()

message(STATUS "Checking internet connection...")
file(DOWNLOAD "https://librealsense.intel.com/Releases/connectivity_check" "${CMAKE_CURRENT_SOURCE_DIR}/connectivity_check" SHOW_PROGRESS TIMEOUT 5 STATUS status)
list (FIND status "\"No error\"" _index)
if (${_index} EQUAL -1)
    message(STATUS "Failed to identify Internet connection")
    set(INTERNET_CONNECTION OFF)
else()
    message(STATUS "Internet connection identified")
    set(INTERNET_CONNECTION ON)
endif()
