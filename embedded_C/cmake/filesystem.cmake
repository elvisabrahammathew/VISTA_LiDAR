# Test actual header + link support rather than assuming an OS/compiler version.
include(CheckCXXSourceCompiles)
set(vista_filesystem_probe "
#include <filesystem>
#include <fstream>
int main() {
    auto p = std::filesystem::path(\".\").lexically_normal();
    std::error_code ec;
    auto s = std::filesystem::space(p, ec);
    std::ifstream input(p);
    return s.capacity == 0;
}")
if(NOT VISTA_FORCE_PORTABLE_FILESYSTEM)
    check_cxx_source_compiles("${vista_filesystem_probe}" VISTA_HAS_STD_FILESYSTEM)
    if(NOT VISTA_HAS_STD_FILESYSTEM AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        set(vista_saved_required_libraries "${CMAKE_REQUIRED_LIBRARIES}")
        set(CMAKE_REQUIRED_LIBRARIES "${CMAKE_REQUIRED_LIBRARIES};stdc++fs")
        check_cxx_source_compiles("${vista_filesystem_probe}" VISTA_HAS_STD_FILESYSTEM_EXTRA_LIB)
        set(CMAKE_REQUIRED_LIBRARIES "${vista_saved_required_libraries}")
    endif()
endif()
if(NOT VISTA_FORCE_PORTABLE_FILESYSTEM AND
   (VISTA_HAS_STD_FILESYSTEM OR VISTA_HAS_STD_FILESYSTEM_EXTRA_LIB))
    if(NOT VISTA_HAS_STD_FILESYSTEM)
        target_link_libraries(vista_edge_dependencies INTERFACE stdc++fs)
    endif()
    message(STATUS "VISTA filesystem backend: std::filesystem")
else()
    set(vista_filesystem_root "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/filesystem/include")
    if(NOT EXISTS "${vista_filesystem_root}/ghc/filesystem.hpp")
        message(FATAL_ERROR "Portable filesystem submodule missing. Run: git submodule update --init --recursive")
    endif()
    target_include_directories(vista_edge_core SYSTEM PRIVATE "${vista_filesystem_root}")
    target_include_directories(vista_edge_dependencies SYSTEM INTERFACE "${vista_filesystem_root}")
    target_compile_definitions(vista_edge_core PRIVATE VISTA_USE_PORTABLE_FILESYSTEM=1)
    target_compile_definitions(vista_edge_dependencies INTERFACE VISTA_USE_PORTABLE_FILESYSTEM=1)
    message(STATUS "VISTA filesystem backend: ghc::filesystem (legacy-compatible)")
endif()
