cmake_minimum_required(VERSION 3.16)

# Keep CMake's per-target CRT selection active in older SDK subprojects too.
cmake_policy(SET CMP0091 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0091 NEW)

project(vista_edge VERSION 0.1.0 LANGUAGES CXX)

if(MSVC)
    # Release is self-contained (/MT); keep Debug's existing /MDd ABI and
    # /MD for the other configurations. Set BEFORE creating any library so
    # VISTA objects, Point-LIO, RealSense and tests all use the same CRT.
    set(CMAKE_MSVC_RUNTIME_LIBRARY
        "MultiThreaded$<$<CONFIG:Debug>:Debug>$<$<NOT:$<CONFIG:Release>>:DLL>")
endif()

option(VISTA_BUILD_TESTS "Build the C++ unit tests" ON)
option(VISTA_ENABLE_REALSENSE "Build RealSense L515 support" ON)
option(VISTA_FORCE_PORTABLE_FILESYSTEM
    "Use the portable filesystem backend instead of the detected standard backend" OFF)

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
        # /FS serializes access to the shared PDB and keeps parallel Visual
        # Studio builds from failing with C1041.
        target_compile_options(${target_name} PRIVATE /W4 /permissive- /EHsc /FS)
    else()
        target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

set(VISTA_CORE_SOURCES
    config.cpp
    1_Platform/compat/filesystem.cpp
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
    2_Transport/messaging/websocket_server.cpp
    2_Transport/storage/local.cpp
    2_Transport/storage/room_map_storage.cpp
    2_Transport/storage/room_map_tiles.cpp
    3_Devices/lidars/lidar.cpp
    3_Devices/lidars/quanergym8/quanergym8.cpp
    3_Devices/lidars/realsensel515/realsensel515.cpp
    3_Devices/lidars/unitree4d/unitree4d.cpp
    3_Devices/lidars/unitree4d/unitree_protocol.cpp
    4_Applications/grafana_bridge/grafana_bridge.cpp
    4_Applications/logger/logger.cpp
    4_Applications/mapping/room_map/room_map.cpp
    4_Applications/mapping/room_map/room_map_websocket.cpp
    4_Applications/perception/foreground/foreground_websocket.cpp
    4_Applications/mapping/lio/lio.cpp
    4_Applications/mapping/lio/lio_adapter.cpp
    4_Applications/monitoring/system_monitor.cpp
    4_Applications/pointcloud_websocket/pointcloud_websocket.cpp
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
# Ship recoverable defaults inside the binary, not a source-tree runtime path.
# Track the TXT so editing defaults also regenerates the header on the next build.
# GrafanaSecret.txt is deliberately NOT read or embedded here.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/config/DeviceConfig.defaults.txt")
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/config/DeviceConfig.defaults.txt"
    VISTA_DEFAULT_DEVICE_CONFIG)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # First-run recovery must use the same Linux serial default as publish/.
    string(REGEX REPLACE "SerialPort\\(unitree-l2\\):[^\r\n]*"
        "SerialPort(unitree-l2): /dev/ttyACM0"
        VISTA_DEFAULT_DEVICE_CONFIG "${VISTA_DEFAULT_DEVICE_CONFIG}")
endif()
string(FIND "${VISTA_DEFAULT_DEVICE_CONFIG}" ")VISTA_CONFIG\"" VISTA_CONFIG_DELIMITER)
if(NOT VISTA_CONFIG_DELIMITER EQUAL -1)
    message(FATAL_ERROR "config/DeviceConfig.defaults.txt contains the reserved raw-string delimiter")
endif()
configure_file(cmake/default_device_config.hpp.in
    "${CMAKE_CURRENT_BINARY_DIR}/generated/default_device_config.hpp" @ONLY)
unset(VISTA_DEFAULT_DEVICE_CONFIG)
target_include_directories(vista_edge_core PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/generated")

target_include_directories(vista_edge_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
# Keep shared link/usage requirements in a separate INTERFACE target. Both
# application and tests reuse the same objects and dependencies on every platform.
add_library(vista_edge_dependencies INTERFACE)
target_include_directories(vista_edge_dependencies INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
# Keep the established repository-level publish/data layout when building in
# VISTA. A standalone copy of embedded_C writes both folders inside itself.
get_filename_component(VISTA_PARENT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
if(EXISTS "${VISTA_PARENT_ROOT}/publish/README.md" AND
   EXISTS "${VISTA_PARENT_ROOT}/third_party/README.md")
    set(VISTA_REPOSITORY_ROOT "${VISTA_PARENT_ROOT}")
else()
    set(VISTA_REPOSITORY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
endif()
include(cmake/filesystem.cmake)
# Keep captured data beside the selected publish root on every platform.
target_compile_definitions(vista_edge_core PRIVATE
    VISTA_DATA_ROOT="${VISTA_REPOSITORY_ROOT}/data")
find_package(Threads REQUIRED)
target_link_libraries(vista_edge_dependencies INTERFACE Threads::Threads)
if(UNIX)
    target_compile_options(vista_edge_core PRIVATE -pthread)
endif()
include(cmake/point_lio.cmake)
target_link_libraries(vista_edge_dependencies INTERFACE vista_point_lio)

if(WIN32)
    target_link_libraries(vista_edge_dependencies INTERFACE ws2_32 psapi)
endif()

# All C++ build dependencies are versioned source below embedded_C/library.
set(VISTA_LIBRARY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/library")
set(VISTA_THIRD_PARTY_ROOT
    "${VISTA_LIBRARY_ROOT}"
    CACHE PATH "C++ build dependency source directory")

# Migrate the two old default paths without replacing a user's custom SDK path.
set(VISTA_OLD_SHARED_ROOT "${VISTA_PARENT_ROOT}/third_party")
set(VISTA_OLD_EMBEDDED_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
if(VISTA_THIRD_PARTY_ROOT STREQUAL VISTA_OLD_SHARED_ROOT OR
   VISTA_THIRD_PARTY_ROOT STREQUAL VISTA_OLD_EMBEDDED_ROOT)
    set(VISTA_THIRD_PARTY_ROOT
        "${VISTA_LIBRARY_ROOT}"
        CACHE PATH "C++ build dependency source directory" FORCE)
    message(STATUS "Migrated build dependencies to ${VISTA_THIRD_PARTY_ROOT}")
endif()

set(VISTA_REALSENSE_SOURCE_DIR
    "${VISTA_THIRD_PARTY_ROOT}/librealsense"
    CACHE PATH "Path to the bundled librealsense source")

# Existing build directories may still cache the former vendored-source path.
# Migrate only that known default and preserve any intentional custom path.
set(VISTA_REALSENSE_LEGACY_SOURCE_DIR
    "${VISTA_OLD_SHARED_ROOT}/realsense/librealsense-2.50.0")
set(VISTA_REALSENSE_EMBEDDED_SOURCE_DIR
    "${VISTA_OLD_EMBEDDED_ROOT}/librealsense")
if((VISTA_REALSENSE_SOURCE_DIR STREQUAL VISTA_REALSENSE_LEGACY_SOURCE_DIR OR
    VISTA_REALSENSE_SOURCE_DIR STREQUAL VISTA_REALSENSE_EMBEDDED_SOURCE_DIR OR
    VISTA_REALSENSE_SOURCE_DIR STREQUAL "${VISTA_OLD_SHARED_ROOT}/librealsense") AND
   EXISTS "${VISTA_THIRD_PARTY_ROOT}/librealsense/CMakeLists.txt")
    set(VISTA_REALSENSE_SOURCE_DIR
        "${VISTA_THIRD_PARTY_ROOT}/librealsense"
        CACHE PATH "Path to the bundled librealsense source" FORCE)
    message(STATUS "Migrated cached librealsense path to embedded_C/library")
endif()

if(VISTA_ENABLE_REALSENSE)
    if(NOT EXISTS "${VISTA_REALSENSE_SOURCE_DIR}/CMakeLists.txt")
        message(FATAL_ERROR
            "Missing bundled librealsense source at ${VISTA_REALSENSE_SOURCE_DIR}. "
            "Restore embedded_C/library/librealsense from the repository.")
    endif()

    # Build only the static SDK required by this application. This removes the
    # need to store or deploy a platform-specific realsense2 DLL/shared object.
    set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build librealsense as a static library" FORCE)
    # The CMake runtime property above selects the CRT per configuration;
    # do not use the SDK's legacy flag rewriting across all configurations.
    set(BUILD_WITH_STATIC_CRT OFF CACHE BOOL "CRT selected per configuration by VISTA" FORCE)
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
        "${CMAKE_CURRENT_BINARY_DIR}/library/librealsense"
        EXCLUDE_FROM_ALL)

    if(NOT TARGET realsense2)
        message(FATAL_ERROR "The vendored librealsense source did not create target realsense2")
    endif()

    target_compile_definitions(vista_edge_core PUBLIC VISTA_HAS_REALSENSE=1)
    target_compile_definitions(vista_edge_dependencies INTERFACE VISTA_HAS_REALSENSE=1)
    target_include_directories(vista_edge_core SYSTEM PRIVATE
        "${VISTA_REALSENSE_SOURCE_DIR}/include")
    target_link_libraries(vista_edge_dependencies INTERFACE realsense2)
    message(STATUS
        "librealsense: building bundled static source from "
        "${VISTA_REALSENSE_SOURCE_DIR} (RSUSB=${VISTA_REALSENSE_FORCE_RSUSB_BACKEND})")
else()
    target_compile_definitions(vista_edge_core PUBLIC VISTA_HAS_REALSENSE=0)
    target_compile_definitions(vista_edge_dependencies INTERFACE VISTA_HAS_REALSENSE=0)
    message(STATUS
        "librealsense support is disabled; the L515 backend will return an "
        "unsupported error")
endif()

add_subdirectory(cmake/view_processing view_processing_target)
add_executable(vista_edge main.cpp)
vista_enable_warnings(vista_edge)
target_sources(vista_edge PRIVATE $<TARGET_OBJECTS:vista_edge_core>)
target_sources(vista_edge PRIVATE $<TARGET_OBJECTS:vista_view_processing>)
target_link_libraries(vista_edge PRIVATE vista_edge_dependencies)

# Runtime settings belong to the executable directory and are user-owned.
# Never copy DeviceConfig.txt or GrafanaSecret.txt during a build. First startup
# exclusively creates missing files; existing settings/tokens remain untouched.

# Windows Release links directly into publish/. Debug paths, Linux output
# paths and all acquisition/LIO/mapping options remain unchanged.
include(cmake/publish.cmake)

if(VISTA_BUILD_TESTS)
    enable_testing()
    add_executable(vista_edge_tests
        unittest/test_main.cpp
        unittest/model_messages_test.cpp
        unittest/application_test/logger_test.cpp
        unittest/application_test/room_map_test.cpp
        unittest/application_test/gaussian_map_test.cpp
        unittest/application_test/foreground_test.cpp
        unittest/application_test/lio_test.cpp
        unittest/application_test/grafana_bridge_test.cpp
        unittest/application_test/pointcloud_processing_test.cpp
        unittest/application_test/system_monitor_test.cpp
        unittest/application_test/pointcloud_websocket_test.cpp
        unittest/config_test.cpp
        unittest/platform_test/filesystem_test.cpp
        unittest/device_test/quanergym8_test.cpp
        unittest/device_test/realsensel515_test.cpp
        unittest/device_test/unitree4d_test.cpp
        unittest/platform_test/pubsub_test.cpp
        unittest/platform_test/message_bus_test.cpp
        unittest/platform_test/threading_test.cpp
        unittest/platform_test/workers_test.cpp
        unittest/transport_test/ethernet_test.cpp
        unittest/transport_test/librealsense_usb_test.cpp
        unittest/transport_test/websocket_server_test.cpp
        unittest/transport_test/room_map_tiles_test.cpp
        unittest/transport_test/room_map_websocket_test.cpp
    )
    vista_enable_warnings(vista_edge_tests)
    target_sources(vista_edge_tests PRIVATE $<TARGET_OBJECTS:vista_edge_core>)
    target_sources(vista_edge_tests PRIVATE $<TARGET_OBJECTS:vista_view_processing>)
    target_link_libraries(vista_edge_tests PRIVATE vista_edge_dependencies)
    add_test(NAME vista_edge_tests COMMAND vista_edge_tests)
    add_test(NAME vista_publish_packaging
        COMMAND "${CMAKE_COMMAND}"
            "-DVISTA_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
            "-DVISTA_TEST_ROOT=${CMAKE_CURRENT_BINARY_DIR}/publish-test"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/test_publish.cmake")
endif()
