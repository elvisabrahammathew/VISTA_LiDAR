# Called by publish_package.cmake only for a Windows x64 Release.
get_filename_component(destination "${VISTA_PUBLISH_DIR}" ABSOLUTE)
if(NOT VISTA_PUBLISH_METADATA_DIR)
    get_filename_component(publish_root "${destination}" DIRECTORY)
    set(VISTA_PUBLISH_METADATA_DIR "${publish_root}/build-info/windows-x64")
endif()
get_filename_component(metadata "${VISTA_PUBLISH_METADATA_DIR}" ABSOLUTE)
if(metadata STREQUAL destination)
    message(FATAL_ERROR "Windows build metadata must be outside its three-file app folder")
endif()
file(MAKE_DIRECTORY "${destination}" "${metadata}")
file(LOCK "${metadata}/.publish.lock" GUARD PROCESS TIMEOUT 30)

# Dummy-byte packaging fixtures explicitly skip this; real builds may not ship
# an executable that still needs the Microsoft redistributable DLLs.
if(NOT VISTA_SKIP_WINDOWS_DEPENDENCY_CHECK)
    if(NOT VISTA_DUMPBIN OR NOT EXISTS "${VISTA_DUMPBIN}")
        message(FATAL_ERROR "Windows publishing requires the MSVC dumpbin tool")
    endif()
    execute_process(COMMAND "${VISTA_DUMPBIN}" /nologo /dependents "${VISTA_EXECUTABLE}"
        OUTPUT_VARIABLE dependencies ERROR_VARIABLE inspect_error RESULT_VARIABLE inspect_result)
    if(NOT inspect_result EQUAL 0)
        message(FATAL_ERROR "Cannot inspect Windows dependencies: ${inspect_error}")
    endif()
    string(TOLOWER "${dependencies}" dependencies_lower)
    if(dependencies_lower MATCHES "(msvcp[0-9]|vcruntime[0-9]|concrt[0-9]|ucrtbase|api-ms-win-crt-)")
        message(FATAL_ERROR "Windows Release still imports a CRT DLL; rebuild all linked targets with /MT")
    endif()
    file(WRITE "${metadata}/DEPENDENCIES.txt" "${dependencies}")
endif()

# Normally the linker already produced this exact destination file. Keeping
# the separate-source path permits explicit packaging/test fixtures too.
get_filename_component(executable "${VISTA_EXECUTABLE}" ABSOLUTE)
set(published "${destination}/vista_edge.exe")
if(NOT executable STREQUAL published)
    execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_if_different
        "${executable}" "${published}" RESULT_VARIABLE copy_result)
    if(NOT copy_result EQUAL 0)
        message(FATAL_ERROR "Cannot update Windows EXE; stop the published app and retry")
    endif()
endif()

# Never read any developer's runtime config/token. Missing files get safe
# templates; existing files, even empty or invalid ones, remain user-owned.
file(READ "${VISTA_DEFAULT_CONFIG}" defaults)
file(WRITE "${metadata}/DeviceConfig.defaults.txt" "${defaults}")
if(NOT EXISTS "${destination}/DeviceConfig.txt")
    file(WRITE "${destination}/DeviceConfig.txt" "${defaults}")
endif()
if(NOT EXISTS "${destination}/GrafanaSecret.txt")
    file(WRITE "${destination}/GrafanaSecret.txt"
        "# Grafana service-account token. Keep this file private; do not commit it.\n"
        "# Paste your token after the colon, then restart the application.\nGrafanaToken: \n")
endif()
file(SHA256 "${published}" executable_hash)
file(WRITE "${metadata}/BUILD_INFO.txt"
    "Program: VISTA Edge\nPlatform: windows-x64\nConfiguration: Release\n"
    "Build OS: ${VISTA_BUILD_OS}\nCompiler: ${VISTA_BUILD_COMPILER}\n"
    "MSVC runtime: static (/MT); Windows system DLLs are provided by the OS.\n"
    "RealSense support: ${VISTA_REALSENSE}\nExecutable: vista_edge.exe\n"
    "SHA256: ${executable_hash}\n"
    "Source: repository working tree at build time (including uncommitted edits).\n"
    "Windows Release linker output is publish/windows-x64/vista_edge.exe.\n")
message(STATUS "Windows Release ready: ${published} (EXE + two user-owned TXT configs)")
