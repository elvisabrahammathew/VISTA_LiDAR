# Port only generated build-tree copies; third_party/point_lio_unilidar is read-only.
# The numerical target requires Eigen/Boost.Preprocessor, not ROS/PCL/Python.
set(VISTA_POINT_LIO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/point_lio_unilidar")
set(VISTA_EIGEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/eigen")
set(VISTA_BOOST_PP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/boost_preprocessor/include")
foreach(required
    "${VISTA_POINT_LIO_ROOT}/src/Estimator.h"
    "${VISTA_POINT_LIO_ROOT}/src/Estimator.cpp"
    "${VISTA_POINT_LIO_ROOT}/include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp"
    "${VISTA_POINT_LIO_ROOT}/include/ikd-Tree/ikd_Tree.cpp"
    "${VISTA_EIGEN_ROOT}/Eigen/Core"
    "${VISTA_BOOST_PP_ROOT}/boost/preprocessor/seq.hpp")
    if(NOT EXISTS "${required}")
        message(FATAL_ERROR "Missing ${required}; initialize the Point-LIO/Eigen/Boost submodules")
    endif()
endforeach()
set(VISTA_LIO_PORT "${CMAKE_CURRENT_BINARY_DIR}/point_lio_port")
file(MAKE_DIRECTORY "${VISTA_LIO_PORT}")
file(GLOB_RECURSE lio_upstream_headers "${VISTA_POINT_LIO_ROOT}/include/IKFoM/IKFoM_toolkit/*.hpp")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${lio_upstream_headers}
    "${VISTA_POINT_LIO_ROOT}/include/ikd-Tree/ikd_Tree.cpp"
    "${VISTA_POINT_LIO_ROOT}/include/ikd-Tree/ikd_Tree.h"
    "${VISTA_POINT_LIO_ROOT}/src/Estimator.h"
    "${VISTA_POINT_LIO_ROOT}/src/Estimator.cpp")
file(COPY "${VISTA_POINT_LIO_ROOT}/include/IKFoM/IKFoM_toolkit" DESTINATION "${VISTA_LIO_PORT}")
set(kf "${VISTA_LIO_PORT}/IKFoM_toolkit/esekfom/esekfom.hpp")
file(READ "${kf}" code)
string(REPLACE "#include <boost/bind.hpp>" "#include <4_Applications/mapping/lio/core/portable_types.hpp>" code "${code}")
file(WRITE "${kf}" "${code}")
set(math "${VISTA_LIO_PORT}/IKFoM_toolkit/mtk/src/mtkmath.hpp")
file(READ "${math}" code)
string(REPLACE "#include <boost/math/tools/precision.hpp>" "#include <limits>" code "${code}")
string(REPLACE "boost::math::tools::epsilon<scalar>()" "std::numeric_limits<scalar>::epsilon()" code "${code}")
file(WRITE "${math}" "${code}")

# Copy the real upstream manifolds and IMU-as-output dynamics. Fail closed if
# their layout changes; do not silently fall back to an old estimator.
function(point_lio_extract source begin_marker end_marker output_name)
    string(FIND "${source}" "${begin_marker}" begin_index)
    string(FIND "${source}" "${end_marker}" end_index)
    if(begin_index LESS 0 OR end_index LESS 0 OR end_index LESS begin_index)
        message(FATAL_ERROR "Point-LIO Estimator layout changed; review portable model extraction")
    endif()
    math(EXPR length "${end_index} - ${begin_index}")
    string(SUBSTRING "${source}" ${begin_index} ${length} fragment)
    set(${output_name} "${fragment}" PARENT_SCOPE)
endfunction()
file(READ "${VISTA_POINT_LIO_ROOT}/src/Estimator.h" estimator_header)
point_lio_extract("${estimator_header}" "typedef MTK::vect<3, double> vect3;" "extern esekfom::esekf<state_input" models)
file(READ "${VISTA_POINT_LIO_ROOT}/src/Estimator.cpp" estimator_source)
point_lio_extract("${estimator_source}" "Eigen::Matrix<double, 30, 1> get_f_output" "Eigen::Matrix<double, 24, 24> df_dx_input" dynamics)
point_lio_extract("${estimator_source}" "Eigen::Matrix<double, 30, 30> df_dx_output" "// Eigen::Matrix<double, 30, 15> df_dw_output" jacobian)
file(WRITE "${VISTA_LIO_PORT}/point_lio_models.hpp"
    "// Generated from point_lio_unilidar/src/Estimator.{h,cpp}; see upstream LICENSE.\n#pragma once\n#include <IKFoM_toolkit/esekfom/esekfom.hpp>\nnamespace vista::application::point_lio_native {\n${models}\n${dynamics}\n${jacobian}\n}\n")
foreach(name ikd_Tree.h ikd_Tree.cpp)
    file(READ "${VISTA_POINT_LIO_ROOT}/include/ikd-Tree/${name}" code)
    string(REPLACE "#include <pcl/point_types.h>" "#include <4_Applications/mapping/lio/core/portable_types.hpp>" code "${code}")
    string(REPLACE "#include <pthread.h>" "" code "${code}")
    string(REPLACE "#include <unistd.h>" "" code "${code}")
    string(REPLACE "pcl::" "vista::lio_port::" code "${code}")
    # A single owner performs bounded synchronous rebuilds, without POSIX
    # background threads or million-entry operation queues.
    string(REPLACE "#define Multi_Thread_Rebuild_Point_Num 1500" "#define Multi_Thread_Rebuild_Point_Num 2147483647" code "${code}")
    string(REPLACE "#define Q_LEN 1000000" "#define Q_LEN 1" code "${code}")
    string(REPLACE "pthread_create(&rebuild_thread, NULL, multi_thread_ptr, (void *)this);" "/* Rebuilt synchronously by the LIO worker. */" code "${code}")
    string(REPLACE [=[printf("Multi thread started \n");]=] "/* No background rebuild thread is started. */" code "${code}")
    foreach(pair "pthread_mutex_init|mutex_init" "pthread_mutex_destroy|mutex_destroy"
                 "pthread_mutex_trylock|mutex_trylock" "pthread_mutex_lock|mutex_lock"
                 "pthread_mutex_unlock|mutex_unlock" "pthread_create|thread_create"
                 "pthread_join|thread_join" "usleep|sleep_us"
                 "pthread_mutex_t|mutex" "pthread_t|thread")
        string(REPLACE "|" ";" parts "${pair}")
        list(GET parts 0 from)
        list(GET parts 1 to)
        string(REPLACE "${from}" "vista::lio_port::${to}" code "${code}")
    endforeach()
    file(WRITE "${VISTA_LIO_PORT}/${name}" "${code}")
endforeach()
set(VISTA_APPLICATION_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
add_subdirectory(cmake/lio point_lio_target)
message(STATUS "LIO backend: Point-LIO / Unitree IMU-as-output (ROS-free)")
