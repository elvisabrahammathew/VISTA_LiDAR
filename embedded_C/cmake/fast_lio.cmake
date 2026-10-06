# FAST-LIO upstream remains read-only. Only generated build-tree copies are ported.
# No ROS, Livox driver, PCL, OpenMP or Python runtime is required by this target.
set(VISTA_FAST_LIO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/fast_lio")
set(VISTA_EIGEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/eigen")
set(VISTA_BOOST_PP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/boost_preprocessor/include")
foreach(required
    "${VISTA_FAST_LIO_ROOT}/include/ikd-Tree/ikd_Tree.cpp"
    "${VISTA_EIGEN_ROOT}/Eigen/Core"
    "${VISTA_BOOST_PP_ROOT}/boost/preprocessor/seq.hpp")
    if(NOT EXISTS "${required}")
        message(FATAL_ERROR "Missing ${required}; run git submodule update --init --recursive")
    endif()
endforeach()
set(VISTA_LIO_PORT "${CMAKE_CURRENT_BINARY_DIR}/fast_lio_port")
file(MAKE_DIRECTORY "${VISTA_LIO_PORT}")
# Reconfigure whenever the pinned source or adapter changes.
file(GLOB_RECURSE lio_upstream_headers "${VISTA_FAST_LIO_ROOT}/include/IKFoM_toolkit/*.hpp")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${lio_upstream_headers}
    "${VISTA_FAST_LIO_ROOT}/include/ikd-Tree/ikd_Tree.cpp"
    "${VISTA_FAST_LIO_ROOT}/include/ikd-Tree/ikd_Tree.h")
file(COPY "${VISTA_FAST_LIO_ROOT}/include/IKFoM_toolkit" DESTINATION "${VISTA_LIO_PORT}")
# boost/bind is an unused include; math::epsilon equals the standard epsilon.
set(kf "${VISTA_LIO_PORT}/IKFoM_toolkit/esekfom/esekfom.hpp")
file(READ "${kf}" code)
string(REPLACE "#include <boost/bind.hpp>" "#include <4_Applications/mapping/lio/core/portable_types.hpp>" code "${code}")
string(REPLACE "omp_get_wtime()" "vista::lio_port::wall_time()" code "${code}")
file(WRITE "${kf}" "${code}")
set(math "${VISTA_LIO_PORT}/IKFoM_toolkit/mtk/src/mtkmath.hpp")
file(READ "${math}" code)
string(REPLACE "#include <boost/math/tools/precision.hpp>" "#include <limits>" code "${code}")
string(REPLACE "boost::math::tools::epsilon<scalar>()" "std::numeric_limits<scalar>::epsilon()" code "${code}")
file(WRITE "${math}" "${code}")
foreach(name ikd_Tree.h ikd_Tree.cpp)
    file(READ "${VISTA_FAST_LIO_ROOT}/include/ikd-Tree/${name}" code)
    string(REPLACE "#include <pcl/point_types.h>" "#include <4_Applications/mapping/lio/core/portable_types.hpp>" code "${code}")
    string(REPLACE "#include <pthread.h>" "" code "${code}")
    string(REPLACE "#include <unistd.h>" "" code "${code}")
    string(REPLACE "pcl::" "vista::lio_port::" code "${code}")
    # A single owner performs local-map updates. Disable hidden background
    # rebuild threads/queues; synchronous rebuilds retain the upstream algorithm.
    string(REPLACE "#define Multi_Thread_Rebuild_Point_Num 1500" "#define Multi_Thread_Rebuild_Point_Num 2147483647" code "${code}")
    string(REPLACE "#define Q_LEN 1000000" "#define Q_LEN 1" code "${code}")
    string(REPLACE "pthread_create(&rebuild_thread, NULL, multi_thread_ptr, (void *)this);" "/* Rebuilds run synchronously in the owning LIO worker. */" code "${code}")
    # The disabled background thread must not announce a startup on every tree.
    string(REPLACE [=[printf("Multi thread started \n");]=] "/* Omit the disabled background-thread startup log. */" code "${code}")
    foreach(pair "pthread_mutex_init|mutex_init"
                 "pthread_mutex_destroy|mutex_destroy" "pthread_mutex_trylock|mutex_trylock"
                 "pthread_mutex_lock|mutex_lock" "pthread_mutex_unlock|mutex_unlock"
                 "pthread_create|thread_create" "pthread_join|thread_join" "usleep|sleep_us"
                 "pthread_mutex_t|mutex" "pthread_t|thread")
        string(REPLACE "|" ";" parts "${pair}")
        list(GET parts 0 from)
        list(GET parts 1 to)
        string(REPLACE "${from}" "vista::lio_port::${to}" code "${code}")
    endforeach()
    file(WRITE "${VISTA_LIO_PORT}/${name}" "${code}")
endforeach()
set(VISTA_APPLICATION_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
# A separate directory confines the Debug numerical optimization to this
# target, including compatibility with CMake versions predating CMP0184.
add_subdirectory(cmake/lio fast_lio_target)
