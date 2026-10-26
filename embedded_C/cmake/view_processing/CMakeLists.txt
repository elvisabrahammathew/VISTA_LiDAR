# Keep symbols and the Debug ABI, but optimize these two CPU-heavy optional
# branches. /RTC and /Od changes stay local; acquisition, mapping, transport,
# controller code and tests retain their normal Debug settings.
if(MSVC)
    string(REGEX REPLACE "/RTC[1scu]+" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
    string(REPLACE "/Od" "/O2" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
endif()
add_library(vista_view_processing OBJECT
    "${PROJECT_SOURCE_DIR}/4_Applications/mapping/gaussian_map/gaussian_map.cpp"
    "${PROJECT_SOURCE_DIR}/4_Applications/perception/foreground/foreground.cpp")
target_link_libraries(vista_view_processing PRIVATE vista_edge_dependencies)
vista_enable_warnings(vista_view_processing)
if(MSVC)
    target_compile_options(vista_view_processing PRIVATE /Ob2)
else()
    target_compile_options(vista_view_processing PRIVATE $<$<CONFIG:Debug>:-O2>)
endif()
