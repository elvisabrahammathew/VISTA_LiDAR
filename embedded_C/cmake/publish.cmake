# Windows Release uses a minimal app folder and links directly into publish/.
# Debug and Linux keep their existing output paths and packaging behavior.
option(VISTA_AUTO_PUBLISH "Refresh repo publish/<platform> after Release builds" ON)
if(NOT VISTA_AUTO_PUBLISH)
    return()
endif()
if(WIN32 AND NOT MSVC)
    message(STATUS "Minimal Windows publishing requires MSVC; normal build output is unchanged")
    return()
endif()

if(WIN32 AND CMAKE_SIZEOF_VOID_P EQUAL 8 AND
   VISTA_SYSTEM_PROCESSOR_LOWER MATCHES "^(amd64|x86_64|x64)$")
    set(VISTA_PUBLISH_PLATFORM windows-x64)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND
       VISTA_SYSTEM_PROCESSOR_LOWER MATCHES "^(amd64|x86_64|x64)$")
    set(VISTA_PUBLISH_PLATFORM linux-x86_64)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND
       VISTA_SYSTEM_PROCESSOR_LOWER MATCHES "^(aarch64|arm64)$")
    set(VISTA_PUBLISH_PLATFORM linux-arm64)
else()
    message(STATUS "Automatic publish is unavailable for this OS/architecture")
    return()
endif()

set(VISTA_PUBLISH_ROOT "${VISTA_REPOSITORY_ROOT}/publish" CACHE PATH
    "Parent of the architecture-specific runnable Release packages")
set(VISTA_PUBLISH_DIR "${VISTA_PUBLISH_ROOT}/${VISTA_PUBLISH_PLATFORM}")
set(VISTA_PUBLISH_WINDOWS_RUNTIME "")
set(VISTA_PUBLISH_METADATA_DIR "${VISTA_PUBLISH_DIR}")
if(MSVC)
    set(VISTA_PUBLISH_METADATA_DIR "${VISTA_PUBLISH_ROOT}/build-info/${VISTA_PUBLISH_PLATFORM}")
    get_filename_component(vista_linker_dir "${CMAKE_LINKER}" DIRECTORY)
    find_program(VISTA_DUMPBIN NAMES dumpbin HINTS "${vista_linker_dir}" NO_DEFAULT_PATH)
    if(NOT VISTA_DUMPBIN)
        message(FATAL_ERROR "dumpbin is required to verify the single-EXE Windows Release package")
    endif()
    set_target_properties(vista_edge PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${VISTA_PUBLISH_DIR}"
        ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_BINARY_DIR}/Release"
        PDB_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_BINARY_DIR}/Release")
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # Resolve bundled, non-glibc libraries beside the actual executable, even
    # when the clone's absolute path differs from the machine that built it.
    set_property(TARGET vista_edge APPEND PROPERTY BUILD_RPATH "$ORIGIN")
endif()

set(VISTA_PUBLISH_OS "${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_VERSION}")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND EXISTS "/etc/os-release")
    file(STRINGS /etc/os-release vista_os_line REGEX "^PRETTY_NAME=")
    string(REGEX REPLACE "^PRETTY_NAME=" "" VISTA_PUBLISH_OS "${vista_os_line}")
    string(REGEX REPLACE "^\"|\"$" "" VISTA_PUBLISH_OS "${VISTA_PUBLISH_OS}")
endif()
set(VISTA_PUBLISH_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
configure_file("${CMAKE_CURRENT_LIST_DIR}/publish_settings.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/publish_settings.cmake" @ONLY)

set(vista_publish_command "${CMAKE_COMMAND}"
    "-DVISTA_CONFIG=$<CONFIG>"
    "-DVISTA_EXECUTABLE=$<TARGET_FILE:vista_edge>"
    "-DVISTA_SETTINGS=${CMAKE_CURRENT_BINARY_DIR}/publish_settings.cmake"
    -P "${CMAKE_CURRENT_LIST_DIR}/publish_package.cmake")
# Works when Visual Studio builds only the startup target (not ALL_BUILD).
add_custom_command(TARGET vista_edge POST_BUILD
    COMMAND ${vista_publish_command} VERBATIM)
# An explicit build of vista_publish also restores deleted packages without
# forcing a relink. ALL refreshes files after a normal complete Release build.
add_custom_target(vista_publish ALL
    COMMAND ${vista_publish_command}
    DEPENDS vista_edge VERBATIM)
message(STATUS "Release auto-publish: ${VISTA_PUBLISH_DIR} (Debug never replaces it)")
