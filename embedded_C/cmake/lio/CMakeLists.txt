# Numerical code is optimized even during Visual Studio Debug. Keep /Zi symbols
# but remove conflicting /RTC1 and /Od ONLY in this child directory.
if(MSVC)
    string(REGEX REPLACE "/RTC[1scu]+" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
    string(REPLACE "/Od" "/O2" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
endif()
add_library(vista_fast_lio STATIC
    "${VISTA_LIO_PORT}/ikd_Tree.cpp"
    "${VISTA_APPLICATION_ROOT}/4_Applications/mapping/lio/core/lio_engine.cpp")
target_include_directories(vista_fast_lio PRIVATE "${VISTA_APPLICATION_ROOT}")
target_include_directories(vista_fast_lio SYSTEM PRIVATE
    "${VISTA_LIO_PORT}" "${VISTA_EIGEN_ROOT}" "${VISTA_BOOST_PP_ROOT}"
    "${VISTA_FAST_LIO_ROOT}/include")
target_link_libraries(vista_fast_lio PRIVATE Threads::Threads)
if(MSVC)
    target_compile_options(vista_fast_lio PRIVATE /bigobj /EHsc /FS /Ob2)
else()
    target_compile_options(vista_fast_lio PRIVATE -O2)
endif()
