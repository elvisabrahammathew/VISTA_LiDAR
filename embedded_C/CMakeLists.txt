cmake_minimum_required(VERSION 3.16)

project(vista_lidar_cpp VERSION 0.1.0 LANGUAGES CXX)

option(VISTA_BUILD_TESTS "Build the C++ unit tests" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Apply strict warnings only to code owned by this project. Vendored SDKs keep
# their own compiler settings and must not inherit /permissive- or -Wpedantic.
function(vista_enable_warnings target_name)
    if(MSVC)
        target_compile_options(${target_name} PRIVATE /W4 /permissive- /EHsc)
    else()
        target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

set(VISTA_LIBRARY_SOURCES
    config.cpp
    1_Platform/message_bus/message_bus.cpp
    1_Platform/pubsub/pubsub.cpp
    1_Platform/threading/threading.cpp
    1_Platform/workers/workers.cpp
    2_Transport/peripheral/ethernet/ethernet.cpp
    2_Transport/peripheral/librealsense_usb/librealsense_usb.cpp
    2_Transport/storage/local.cpp
    3_Devices/lidars/lidar.cpp
    3_Devices/lidars/quanergym8/quanergym8.cpp
    3_Devices/lidars/realsensel515/realsensel515.cpp
    4_Applications/logger/logger.cpp
    4_Applications/processing/pointcloud/pointcloud_processing.cpp
)

if(WIN32)
    list(APPEND VISTA_LIBRARY_SOURCES 1_Platform/os/window.cpp)
elseif(UNIX AND NOT APPLE)
    list(APPEND VISTA_LIBRARY_SOURCES 1_Platform/os/linux.cpp)
else()
    message(FATAL_ERROR "Only Windows and Linux are currently supported")
endif()

add_library(vista_lidar STATIC ${VISTA_LIBRARY_SOURCES})
vista_enable_warnings(vista_lidar)
target_include_directories(vista_lidar PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
# Keep captured data beside the repository on every build platform.
target_compile_definitions(vista_lidar PRIVATE
    VISTA_DATA_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../data")
find_package(Threads REQUIRED)
target_link_libraries(vista_lidar PUBLIC Threads::Threads)

if(WIN32)
    target_link_libraries(vista_lidar PUBLIC ws2_32)
endif()

# All vendored dependencies live below embedded_C/third_party. The same
# librealsense source tree is compiled for the current Windows/Linux CPU.
set(VISTA_THIRD_PARTY_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party"
    CACHE PATH "Root directory for vendored third-party dependencies")
set(VISTA_REALSENSE_SOURCE_DIR
    "${VISTA_THIRD_PARTY_ROOT}/realsense/librealsense-2.50.0"
    CACHE PATH "Path to the vendored librealsense source tree")
option(VISTA_ENABLE_REALSENSE "Build RealSense L515 support" ON)
option(VISTA_REALSENSE_FORCE_RSUSB_BACKEND
    "Use librealsense's user-space USB backend instead of the native OS backend"
    OFF)

if(VISTA_ENABLE_REALSENSE AND
   EXISTS "${VISTA_REALSENSE_SOURCE_DIR}/CMakeLists.txt")
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

    target_compile_definitions(vista_lidar PUBLIC VISTA_HAS_REALSENSE=1)
    target_include_directories(vista_lidar SYSTEM PRIVATE
        "${VISTA_REALSENSE_SOURCE_DIR}/include")
    target_link_libraries(vista_lidar PRIVATE realsense2)
    message(STATUS "librealsense: building static 2.50.0 from ${VISTA_REALSENSE_SOURCE_DIR}")
else()
    target_compile_definitions(vista_lidar PUBLIC VISTA_HAS_REALSENSE=0)
    message(WARNING
        "Vendored librealsense source was not found or support is disabled; "
        "the L515 backend will return an unsupported error")
endif()

add_executable(vista_capture main.cpp)
vista_enable_warnings(vista_capture)
target_link_libraries(vista_capture PRIVATE vista_lidar)

add_custom_command(TARGET vista_capture POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_CURRENT_SOURCE_DIR}/DeviceConfig.txt"
        "$<TARGET_FILE_DIR:vista_capture>/DeviceConfig.txt")

if(VISTA_BUILD_TESTS)
    enable_testing()
    add_executable(vista_tests
        unittest/test_main.cpp
        unittest/application_test/logger_test.cpp
        unittest/application_test/pointcloud_processing_test.cpp
        unittest/config_test.cpp
        unittest/device_test/quanergym8_test.cpp
        unittest/device_test/realsensel515_test.cpp
        unittest/platform_test/pubsub_test.cpp
        unittest/platform_test/message_bus_test.cpp
        unittest/platform_test/threading_test.cpp
        unittest/platform_test/workers_test.cpp
        unittest/transport_test/ethernet_test.cpp
        unittest/transport_test/librealsense_usb_test.cpp
    )
    vista_enable_warnings(vista_tests)
    target_link_libraries(vista_tests PRIVATE vista_lidar)
    add_test(NAME vista_tests COMMAND vista_tests)
endif()
