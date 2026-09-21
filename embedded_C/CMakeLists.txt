cmake_minimum_required(VERSION 3.16)

project(vista_edge VERSION 0.1.0 LANGUAGES CXX)

option(VISTA_BUILD_TESTS "Build the C++ unit tests" ON)
option(VISTA_ENABLE_REALSENSE "Build RealSense L515 support" ON)

# The same source tree is built natively on Windows x64, Linux x86-64, and
# Linux ARM64. ARM64 defaults to librealsense's user-space USB backend so a
# Jetson deployment does not depend on patched kernel UVC drivers.
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" VISTA_SYSTEM_PROCESSOR_LOWER)
set(VISTA_EXPECTED_ARCHITECTURE "" CACHE STRING
    "Optional architecture guard: x86_64 or arm64")
if(VISTA_EXPECTED_ARCHITECTURE STREQUAL "x86_64" AND
   NOT VISTA_SYSTEM_PROCESSOR_LOWER MATCHES "^(amd64|x86_64|x64)$")
    message(FATAL_ERROR
        "This preset requires x86-64, but CMake detected ${CMAKE_SYSTEM_PROCESSOR}")
elseif(VISTA_EXPECTED_ARCHITECTURE STREQUAL "arm64" AND
       NOT VISTA_SYSTEM_PROCESSOR_LOWER MATCHES "^(aarch64|arm64)$")
    message(FATAL_ERROR
        "This preset requires ARM64, but CMake detected ${CMAKE_SYSTEM_PROCESSOR}")
elseif(NOT VISTA_EXPECTED_ARCHITECTURE STREQUAL "" AND
       NOT VISTA_EXPECTED_ARCHITECTURE MATCHES "^(x86_64|arm64)$")
    message(FATAL_ERROR
        "VISTA_EXPECTED_ARCHITECTURE must be empty, x86_64, or arm64")
endif()
set(VISTA_REALSENSE_RSUSB_DEFAULT OFF)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND
   VISTA_SYSTEM_PROCESSOR_LOWER MATCHES "^(aarch64|arm64)$")
    set(VISTA_REALSENSE_RSUSB_DEFAULT ON)
endif()
option(VISTA_REALSENSE_FORCE_RSUSB_BACKEND
    "Use librealsense's user-space USB backend instead of the native OS backend"
    ${VISTA_REALSENSE_RSUSB_DEFAULT})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# CMake 4 removed its implicit compatibility mode for projects older than
# CMake 3.5. librealsense 2.50.0 still declares an older minimum, so keep the
# SDK build working without editing any vendored third-party source file.
if(CMAKE_VERSION VERSION_GREATER_EQUAL 4.0)
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()

# Apply strict warnings only to code owned by this project. Vendored SDKs keep
# their own compiler settings and must not inherit /permissive- or -Wpedantic.
function(vista_enable_warnings target_name)
    if(MSVC)
        target_compile_options(${target_name} PRIVATE /W4 /permissive- /EHsc)
    else()
        target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

set(VISTA_CORE_SOURCES
    config.cpp
    1_Platform/metrics/runtime_metrics.cpp
    1_Platform/message_bus/message_bus.cpp
    1_Platform/pubsub/pubsub.cpp
    1_Platform/threading/threading.cpp
    1_Platform/workers/workers.cpp
    2_Transport/peripheral/ethernet/ethernet.cpp
    2_Transport/peripheral/serial/serial.cpp
    2_Transport/peripheral/udp/udp.cpp
    2_Transport/peripheral/librealsense_usb/librealsense_usb.cpp
    2_Transport/messaging/http.cpp
    2_Transport/storage/local.cpp
    3_Devices/lidars/lidar.cpp
    3_Devices/lidars/quanergym8/quanergym8.cpp
    3_Devices/lidars/realsensel515/realsensel515.cpp
    3_Devices/lidars/unitree4d/unitree4d.cpp
    3_Devices/lidars/unitree4d/unitree_protocol.cpp
    4_Applications/grafana_bridge/grafana_bridge.cpp
    4_Applications/logger/logger.cpp
    4_Applications/monitoring/system_monitor.cpp
    4_Applications/processing/pointcloud/pointcloud_processing.cpp
)

if(WIN32)
    list(APPEND VISTA_CORE_SOURCES 1_Platform/os/window.cpp)
elseif(UNIX AND NOT APPLE)
    list(APPEND VISTA_CORE_SOURCES 1_Platform/os/linux.cpp)
else()
    message(FATAL_ERROR "Only Windows and Linux are currently supported")
endif()

# Compile VISTA-owned sources once and reuse the objects in the application and
# tests. An OBJECT target does not create a separate vista_edge_core library.
add_library(vista_edge_core OBJECT ${VISTA_CORE_SOURCES})
vista_enable_warnings(vista_edge_core)
target_include_directories(vista_edge_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
# Keep captured data beside the repository on every build platform.
target_compile_definitions(vista_edge_core PRIVATE
    VISTA_DATA_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../data")
find_package(Threads REQUIRED)
target_link_libraries(vista_edge_core PUBLIC Threads::Threads)

if(WIN32)
    target_link_libraries(vista_edge_core PUBLIC ws2_32 psapi)
endif()

# All external dependencies live in the repository-level third_party folder as
# Git submodules. C++ and Rust build the same pinned librealsense source commit.
get_filename_component(VISTA_REPOSITORY_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(VISTA_SHARED_THIRD_PARTY_ROOT "${VISTA_REPOSITORY_ROOT}/third_party")
set(VISTA_THIRD_PARTY_ROOT
    "${VISTA_SHARED_THIRD_PARTY_ROOT}"
    CACHE PATH "Repository-level third-party dependency directory")

# Migrate build directories configured before third_party moved to repo root.
get_filename_component(VISTA_LEGACY_THIRD_PARTY_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party" ABSOLUTE)
if(VISTA_THIRD_PARTY_ROOT STREQUAL VISTA_LEGACY_THIRD_PARTY_ROOT AND
   EXISTS "${VISTA_SHARED_THIRD_PARTY_ROOT}/librealsense/CMakeLists.txt")
    set(VISTA_THIRD_PARTY_ROOT
        "${VISTA_SHARED_THIRD_PARTY_ROOT}"
        CACHE PATH "Repository-level third-party dependency directory" FORCE)
    message(STATUS "Migrated third-party root to ${VISTA_THIRD_PARTY_ROOT}")
endif()

set(VISTA_REALSENSE_SOURCE_DIR
    "${VISTA_THIRD_PARTY_ROOT}/librealsense"
    CACHE PATH "Path to the librealsense Git submodule")

# Existing build directories may still cache the former vendored-source path.
# Migrate only that known default and preserve any intentional custom path.
set(VISTA_REALSENSE_LEGACY_SOURCE_DIR
    "${VISTA_THIRD_PARTY_ROOT}/realsense/librealsense-2.50.0")
set(VISTA_REALSENSE_EMBEDDED_SOURCE_DIR
    "${VISTA_LEGACY_THIRD_PARTY_ROOT}/librealsense")
if((VISTA_REALSENSE_SOURCE_DIR STREQUAL VISTA_REALSENSE_LEGACY_SOURCE_DIR OR
    VISTA_REALSENSE_SOURCE_DIR STREQUAL VISTA_REALSENSE_EMBEDDED_SOURCE_DIR) AND
   EXISTS "${VISTA_THIRD_PARTY_ROOT}/librealsense/CMakeLists.txt")
    set(VISTA_REALSENSE_SOURCE_DIR
        "${VISTA_THIRD_PARTY_ROOT}/librealsense"
        CACHE PATH "Path to the librealsense Git submodule" FORCE)
    message(STATUS "Migrated cached librealsense path to the Git submodule")
endif()

if(VISTA_ENABLE_REALSENSE)
    if(NOT EXISTS "${VISTA_REALSENSE_SOURCE_DIR}/CMakeLists.txt")
        message(FATAL_ERROR
            "The librealsense submodule is not initialized at "
            "${VISTA_REALSENSE_SOURCE_DIR}. Run: "
            "git submodule update --init --recursive")
    endif()

    # Build only the static SDK required by this application. This removes the
    # need to store or deploy a platform-specific realsense2 DLL/shared object.
    set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build librealsense as a static library" FORCE)
    set(BUILD_WITH_STATIC_CRT OFF CACHE BOOL "Use the application's MSVC runtime" FORCE)
    set(BUILD_EXAMPLES OFF CACHE BOOL "Disable librealsense examples" FORCE)
    set(BUILD_GRAPHICAL_EXAMPLES OFF CACHE BOOL "Disable graphical examples" FORCE)
    set(BUILD_TOOLS OFF CACHE BOOL "Disable librealsense tools" FORCE)
    set(BUILD_UNIT_TESTS OFF CACHE BOOL "Disable librealsense unit tests" FORCE)
    set(BUILD_INTERNAL_UNIT_TESTS OFF CACHE BOOL "Disable internal SDK tests" FORCE)
    set(BUILD_LEGACY_LIVE_TEST OFF CACHE BOOL "Disable legacy SDK tests" FORCE)
    set(BUILD_WITH_TM2 OFF CACHE BOOL "Disable unused TM2 support" FORCE)
    set(IMPORT_DEPTH_CAM_FW OFF CACHE BOOL "Do not download firmware while configuring" FORCE)
    set(CHECK_FOR_UPDATES OFF CACHE BOOL "Do not perform SDK update checks" FORCE)
    set(BUILD_GLSL_EXTENSIONS OFF CACHE BOOL "Disable OpenGL extensions" FORCE)
    set(BUILD_NETWORK_DEVICE OFF CACHE BOOL "Disable SDK network-device support" FORCE)
    set(BUILD_WITH_CUDA OFF CACHE BOOL "No CUDA processing is required" FORCE)
    set(BUILD_WITH_OPENMP OFF CACHE BOOL "No SDK OpenMP processing is required" FORCE)
    set(ENABLE_CCACHE OFF CACHE BOOL "Do not require ccache" FORCE)
    set(FORCE_RSUSB_BACKEND ${VISTA_REALSENSE_FORCE_RSUSB_BACKEND}
        CACHE BOOL "Select librealsense user-space USB backend" FORCE)

    add_subdirectory(
        "${VISTA_REALSENSE_SOURCE_DIR}"
        "${CMAKE_CURRENT_BINARY_DIR}/third_party/librealsense"
        EXCLUDE_FROM_ALL)

    if(NOT TARGET realsense2)
        message(FATAL_ERROR "The vendored librealsense source did not create target realsense2")
    endif()

    target_compile_definitions(vista_edge_core PUBLIC VISTA_HAS_REALSENSE=1)
    target_include_directories(vista_edge_core SYSTEM PRIVATE
        "${VISTA_REALSENSE_SOURCE_DIR}/include")
    # PUBLIC propagates the SDK link dependency to every target that consumes
    # the VISTA object files; the SDK itself remains a static library.
    target_link_libraries(vista_edge_core PUBLIC realsense2)
    message(STATUS
        "librealsense: building static submodule from "
        "${VISTA_REALSENSE_SOURCE_DIR} (RSUSB=${VISTA_REALSENSE_FORCE_RSUSB_BACKEND})")
else()
    target_compile_definitions(vista_edge_core PUBLIC VISTA_HAS_REALSENSE=0)
    message(STATUS
        "librealsense support is disabled; the L515 backend will return an "
        "unsupported error")
endif()

add_executable(vista_edge main.cpp)
vista_enable_warnings(vista_edge)
target_link_libraries(vista_edge PRIVATE vista_edge_core)

add_custom_command(TARGET vista_edge POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/DeviceConfig.txt"
        "$<TARGET_FILE_DIR:vista_edge>/DeviceConfig.txt")

# GrafanaSecret.txt is intentionally ignored by Git. If it exists locally,
# copy it beside the executable on every build so editing the token never
# requires a clean rebuild or command-line setup.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/GrafanaSecret.txt")
    if(CMAKE_CONFIGURATION_TYPES)
        set(VISTA_GRAFANA_SECRET_OUTPUT_DIR
            "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
        set(VISTA_GRAFANA_SECRET_OUTPUT
            "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/GrafanaSecret.txt")
    else()
        set(VISTA_GRAFANA_SECRET_OUTPUT_DIR
            "${CMAKE_CURRENT_BINARY_DIR}")
        set(VISTA_GRAFANA_SECRET_OUTPUT
            "${CMAKE_CURRENT_BINARY_DIR}/GrafanaSecret.txt")
    endif()
    add_custom_target(vista_grafana_secret ALL
        COMMAND ${CMAKE_COMMAND} -E make_directory
            "${VISTA_GRAFANA_SECRET_OUTPUT_DIR}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/GrafanaSecret.txt"
            "${VISTA_GRAFANA_SECRET_OUTPUT}"
        COMMENT "Copying local Grafana secret"
        VERBATIM)
    add_dependencies(vista_edge vista_grafana_secret)
endif()

if(VISTA_BUILD_TESTS)
    enable_testing()
    add_executable(vista_edge_tests
        unittest/test_main.cpp
        unittest/application_test/logger_test.cpp
        unittest/application_test/grafana_bridge_test.cpp
        unittest/application_test/pointcloud_processing_test.cpp
        unittest/application_test/system_monitor_test.cpp
        unittest/config_test.cpp
        unittest/device_test/quanergym8_test.cpp
        unittest/device_test/realsensel515_test.cpp
        unittest/device_test/unitree4d_test.cpp
        unittest/platform_test/pubsub_test.cpp
        unittest/platform_test/message_bus_test.cpp
        unittest/platform_test/threading_test.cpp
        unittest/platform_test/workers_test.cpp
        unittest/transport_test/ethernet_test.cpp
        unittest/transport_test/librealsense_usb_test.cpp
    )
    vista_enable_warnings(vista_edge_tests)
    target_link_libraries(vista_edge_tests PRIVATE vista_edge_core)
    add_test(NAME vista_edge_tests COMMAND vista_edge_tests)
endif()
