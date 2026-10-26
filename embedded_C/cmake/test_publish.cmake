cmake_minimum_required(VERSION 3.16)
# Run against throwaway fixtures, never the actual repo publish directory.
if(NOT DEFINED VISTA_SOURCE_DIR OR NOT DEFINED VISTA_TEST_ROOT)
    message(FATAL_ERROR "Test requires source and temporary output directories")
endif()
string(RANDOM LENGTH 12 ALPHABET abcdef0123456789 nonce)
set(root "${VISTA_TEST_ROOT}/${nonce} with spaces")
file(MAKE_DIRECTORY "${root}/build" "${root}/templates")
file(WRITE "${root}/build/vista_edge.exe" "release fixture one\n")
file(WRITE "${root}/build/DeviceConfig.txt" "private developer settings\n")
file(WRITE "${root}/build/GrafanaSecret.txt" "private developer token\n")
file(WRITE "${root}/templates/defaults.txt"
    "Lidar: unitree-l2\nSerialPort(unitree-l2): COM3\n")
set(script "${VISTA_SOURCE_DIR}/cmake/publish_package.cmake")
function(publish config platform)
    execute_process(COMMAND "${CMAKE_COMMAND}"
        "-DVISTA_CONFIG=${config}" "-DVISTA_PLATFORM=${platform}"
        "-DVISTA_EXECUTABLE=${root}/build/vista_edge.exe"
        "-DVISTA_DEFAULT_CONFIG=${root}/templates/defaults.txt"
        "-DVISTA_PUBLISH_DIR=${root}/publish/${platform}"
        -DVISTA_SKIP_RUNTIME_SCAN=ON -DVISTA_SKIP_WINDOWS_DEPENDENCY_CHECK=ON -P "${script}"
        RESULT_VARIABLE result OUTPUT_VARIABLE output ERROR_VARIABLE error)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "Packaging failed: ${output}\n${error}")
    endif()
endfunction()
publish(Debug windows-x64)
if(EXISTS "${root}/publish")
    message(FATAL_ERROR "Debug unexpectedly created a publish package")
endif()
publish(Release windows-x64)
set(package "${root}/publish/windows-x64")
file(READ "${package}/DeviceConfig.txt" first_config)
if(NOT first_config MATCHES "COM3")
    message(FATAL_ERROR "Windows config was not initialized from the defaults")
endif()
file(READ "${package}/GrafanaSecret.txt" blank_secret)
if(NOT blank_secret MATCHES "GrafanaToken: \n" OR blank_secret MATCHES "private developer token")
    message(FATAL_ERROR "Windows publisher must create a blank token, never copy a developer token")
endif()
file(WRITE "${package}/DeviceConfig.txt" "user-owned COM9\n")
file(WRITE "${package}/GrafanaSecret.txt" "private-test-token\n")
file(WRITE "${root}/build/vista_edge.exe" "release fixture two\n")
file(WRITE "${root}/templates/defaults.txt"
    "Lidar: unitree-l2\nSerialPort(unitree-l2): COM7\n# updated template\n")
publish(Release windows-x64)
file(READ "${package}/DeviceConfig.txt" config)
file(READ "${package}/GrafanaSecret.txt" secret)
file(READ "${package}/vista_edge.exe" exe)
file(READ "${root}/publish/build-info/windows-x64/DeviceConfig.defaults.txt" refreshed_defaults)
if(NOT config STREQUAL "user-owned COM9\n" OR
   NOT secret STREQUAL "private-test-token\n" OR
   NOT exe STREQUAL "release fixture two\n" OR
   NOT refreshed_defaults MATCHES "updated template")
    message(FATAL_ERROR "Rebuild did not refresh binary and preserve user settings")
endif()
foreach(platform linux-x86_64 linux-arm64)
    publish(Release ${platform})
    file(READ "${root}/publish/${platform}/DeviceConfig.txt" config)
    if(NOT config MATCHES "/dev/ttyACM0" OR config MATCHES "COM3")
        message(FATAL_ERROR "Linux serial-port default is incorrect")
    endif()
endforeach()
file(READ "${root}/publish/build-info/windows-x64/BUILD_INFO.txt" info)
file(SHA256 "${root}/build/vista_edge.exe" expected)
if(NOT info MATCHES "${expected}")
    message(FATAL_ERROR "Published checksum does not match the executable")
endif()
# Restore a damaged package without a relink, but preserve user config/token.
file(WRITE "${package}/vista_edge.exe" "damaged published fixture\n")
publish(Release windows-x64)
file(READ "${package}/vista_edge.exe" repaired)
if(NOT repaired STREQUAL exe)
    message(FATAL_ERROR "Publisher did not repair a damaged executable")
endif()
# A real Windows Release links directly into publish/. This must not attempt
# to copy the EXE over itself or create extra files beside the three app files.
execute_process(COMMAND "${CMAKE_COMMAND}" -DVISTA_CONFIG=Release
    -DVISTA_PLATFORM=windows-x64
    "-DVISTA_EXECUTABLE=${package}/vista_edge.exe"
    "-DVISTA_DEFAULT_CONFIG=${root}/templates/defaults.txt"
    "-DVISTA_PUBLISH_DIR=${package}"
    -DVISTA_SKIP_WINDOWS_DEPENDENCY_CHECK=ON -P "${script}"
    RESULT_VARIABLE direct_result)
if(NOT direct_result EQUAL 0)
    message(FATAL_ERROR "Direct-to-publish EXE packaging failed")
endif()
file(GLOB app_files RELATIVE "${package}" "${package}/*" "${package}/.*")
list(SORT app_files)
if(NOT app_files STREQUAL "DeviceConfig.txt;GrafanaSecret.txt;vista_edge.exe")
    message(FATAL_ERROR "Windows app folder is not exactly three files: ${app_files}")
endif()
file(WRITE "${root}/build/vista_edge.exe" "do not publish non-Release\n")
publish(Debug windows-x64)
publish(RelWithDebInfo windows-x64)
file(READ "${package}/vista_edge.exe" exe_after)
if(NOT exe STREQUAL exe_after)
    message(FATAL_ERROR "Non-Release build replaced the package")
endif()
message(STATUS "Publish checks passed; fixtures retained at ${root}")
