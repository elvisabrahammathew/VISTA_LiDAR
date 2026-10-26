cmake_minimum_required(VERSION 3.16)
# Safe no-op for Debug, RelWithDebInfo and builds without a selected config.
if(NOT VISTA_CONFIG STREQUAL "Release")
    return()
endif()
if(DEFINED VISTA_SETTINGS)
    include("${VISTA_SETTINGS}")
endif()
foreach(required VISTA_PUBLISH_DIR VISTA_EXECUTABLE VISTA_DEFAULT_CONFIG VISTA_PLATFORM)
    if(NOT DEFINED ${required} OR "${${required}}" STREQUAL "")
        message(FATAL_ERROR "Publish requires ${required}")
    endif()
endforeach()
if(NOT VISTA_PLATFORM MATCHES "^(windows-x64|linux-x86_64|linux-arm64)$")
    message(FATAL_ERROR "Unsupported publish platform: ${VISTA_PLATFORM}")
endif()
if(NOT EXISTS "${VISTA_EXECUTABLE}" OR NOT EXISTS "${VISTA_DEFAULT_CONFIG}")
    message(FATAL_ERROR "Cannot publish a missing executable or default config")
endif()
if(VISTA_PLATFORM STREQUAL "windows-x64")
    include("${CMAKE_CURRENT_LIST_DIR}/publish_windows.cmake")
    return()
endif()
get_filename_component(source_dir "${VISTA_EXECUTABLE}" DIRECTORY)
get_filename_component(source_dir "${source_dir}" ABSOLUTE)
get_filename_component(destination "${VISTA_PUBLISH_DIR}" ABSOLUTE)
if(source_dir STREQUAL destination)
    message(FATAL_ERROR "Publish destination must differ from the build directory")
endif()

file(MAKE_DIRECTORY "${destination}")
# Serialize publishers from different build trees targeting the same platform.
# This temporary lock file is ignored by publish/'s Git rules.
file(LOCK "${destination}/.publish.lock" GUARD PROCESS TIMEOUT 30)
function(copy_runtime_file source)
    execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        "${source}" "${destination}/" RESULT_VARIABLE result)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "Cannot publish ${source}; stop the published app and retry")
    endif()
endfunction()
get_filename_component(executable_name "${VISTA_EXECUTABLE}" NAME)
file(SHA256 "${VISTA_EXECUTABLE}" build_hash)
set(binary_current FALSE)
if(EXISTS "${destination}/${executable_name}" AND EXISTS "${destination}/BUILD_INFO.txt")
    file(READ "${destination}/BUILD_INFO.txt" previous_info)
    file(SHA256 "${destination}/${executable_name}" previous_hash)
    string(FIND "${previous_info}" "\nBuild executable SHA256: ${build_hash}\n" source_match)
    string(FIND "${previous_info}" "\nSHA256: ${previous_hash}\n" package_match)
    if(NOT source_match EQUAL -1 AND NOT package_match EQUAL -1)
        set(binary_current TRUE)
    endif()
endif()
if(NOT binary_current)
    copy_runtime_file("${VISTA_EXECUTABLE}")
endif()
if(NOT binary_current AND VISTA_PLATFORM MATCHES "^linux-" AND VISTA_STRIP)
    # Strip the PUBLISHED copy only. Original build binaries/test symbols stay
    # intact, and large SDK debug sections do not enter repository history.
    execute_process(COMMAND "${VISTA_STRIP}" --strip-unneeded
        "${destination}/${executable_name}" RESULT_VARIABLE strip_result)
    if(NOT strip_result EQUAL 0)
        message(FATAL_ERROR "Could not strip published Release executable")
    endif()
endif()

# No source/build DeviceConfig.txt or GrafanaSecret.txt is ever read here.
file(READ "${VISTA_DEFAULT_CONFIG}" defaults)
if(VISTA_PLATFORM MATCHES "^linux-")
    string(REGEX REPLACE "SerialPort\\(unitree-l2\\):[^\r\n]*"
        "SerialPort(unitree-l2): /dev/ttyACM0" defaults "${defaults}")
endif()
file(WRITE "${destination}/DeviceConfig.defaults.txt" "${defaults}")
if(NOT EXISTS "${destination}/DeviceConfig.txt")
    file(WRITE "${destination}/DeviceConfig.txt" "${defaults}")
endif()
file(WRITE "${destination}/GrafanaSecret.example.txt"
    "# Copy to GrafanaSecret.txt beside vista_edge, or let first startup create it.\n"
    "# Keep the real GrafanaSecret.txt private. It is ignored by Git.\nGrafanaToken: \n")

set(runtime_names "")
foreach(library IN LISTS VISTA_RUNTIME_LIBRARIES)
    copy_runtime_file("${library}")
    get_filename_component(name "${library}" NAME)
    list(APPEND runtime_names "${name}")
endforeach()
if(VISTA_PLATFORM MATCHES "^linux-" AND NOT VISTA_SKIP_RUNTIME_SCAN)
    # Only search the TARGET compiler's implicit link directories. CMake's
    # generic runtime scanner can consult host ldconfig during cross builds,
    # which risks putting x86-64 libraries in an ARM64 package.
    set(pending "${VISTA_EXECUTABLE}")
    set(visited "")
    while(pending)
        list(GET pending 0 binary)
        list(REMOVE_AT pending 0)
        execute_process(COMMAND "${VISTA_OBJDUMP}" -p "${binary}"
            OUTPUT_VARIABLE dynamic_info RESULT_VARIABLE inspect_result)
        if(NOT inspect_result EQUAL 0)
            message(FATAL_ERROR "Cannot inspect runtime dependencies: ${binary}")
        endif()
        string(REGEX MATCHALL "NEEDED[ \t]+[^ \t\r\n]+" needed "${dynamic_info}")
        foreach(entry IN LISTS needed)
            string(REGEX REPLACE "^NEEDED[ \t]+" "" name "${entry}")
            if(name MATCHES "^ld-linux" OR
               name MATCHES "^lib(c|m|pthread|dl|rt|resolv|util)\\.so" OR
               name IN_LIST visited)
                continue()
            endif()
            list(APPEND visited "${name}")
            unset(library CACHE)
            unset(library)
            find_file(library NAMES "${name}"
                PATHS ${VISTA_TARGET_LIBRARY_DIRS} NO_DEFAULT_PATH)
            if(NOT library)
                message(FATAL_ERROR "Unresolved target runtime library: ${name}")
            endif()
            copy_runtime_file("${library}")
            list(APPEND runtime_names "${name}")
            list(APPEND pending "${library}")
        endforeach()
    endwhile()
endif()

file(SHA256 "${destination}/${executable_name}" executable_hash)
file(WRITE "${destination}/BUILD_INFO.txt"
    "Program: VISTA Edge\nPlatform: ${VISTA_PLATFORM}\nConfiguration: Release\n"
    "Build OS: ${VISTA_BUILD_OS}\nCompiler: ${VISTA_BUILD_COMPILER}\n"
    "RealSense support: ${VISTA_REALSENSE}\nExecutable: ${executable_name}\n"
    "SHA256: ${executable_hash}\nBuild executable SHA256: ${build_hash}\n"
    "Bundled runtimes: ${runtime_names}\n"
    "Source: repository working tree at build time (including uncommitted edits).\n"
    "See publish/README.md for compatibility, configuration and licenses.\n")
message(STATUS "Published Release: ${destination}/${executable_name}")
