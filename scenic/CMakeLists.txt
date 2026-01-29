cmake_minimum_required(VERSION 3.0.2)
project(scenic)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(Json_INCLUDE_DIRS "/usr/include/jsoncpp")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# apt installable packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
find_package(Ceres REQUIRED)
find_package(TBB REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# my packages
find_package(Clipper REQUIRED)
find_package(lgs REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${GTSAM_INCLUDE_DIR}
    ${yaml-cpp_INCLUDE_DIRS}
    ${TBB_INLCUDE_DIRS}
)

# STICKY VO LIB
add_library(stickyvo SHARED
    src/stickyvo/core.cpp
    src/stickyvo/track_db.cpp
    src/stickyvo/triangulation.cpp
    src/stickyvo/pnp.cpp
    src/stickyvo/local_refine.cpp
    src/stickyvo/bundle_adjustment.cpp
    src/stickyvo/vo_frontend.cpp
)

target_link_libraries(stickyvo PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBRARIES}
    Ceres::ceres
)

# GLIDER LIB
add_library(glider SHARED
    src/glider/parameters.cpp
    src/glider/time.cpp
    src/glider/differential_gps.cpp
    src/glider/odometry.cpp
    src/glider/odometry_with_covariance.cpp
    src/glider/factor_manager.cpp
    src/glider/glider.cpp
)

target_link_libraries(glider PUBLIC
    Eigen3::Eigen
    glog::glog
    gtsam
    gtsam_unstable
    yaml-cpp
    tbb
)

# SCENIC LIB
add_library(${PROJECT_NAME} SHARED
    src/scenic/kmeans.cpp
    src/scenic/params.cpp
    src/scenic/segmentation_processor.cpp
    src/scenic/graphing_processor.cpp
    src/scenic/tracking_processor.cpp
    src/scenic/stitching_processor.cpp
    src/scenic/scenic.cpp
    src/scenic/node.cpp
    src/scenic/graph.cpp
    src/scenic/edge.cpp
    src/scenic/rectifier.cpp
    src/scenic/localization.cpp
    src/scenic/buffer_search_coords.cpp
    src/scenic/traversability.cpp
    src/scenic/homography.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${OpenCV_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    yaml-cpp
    tbb
    glog::glog
    Clipper::clipper
    lgs::lgs
    glider
    stickyvo
)

add_executable(lloyds_test examples/test_lloyds.cpp)
target_link_libraries(lloyds_test
  ${OpenCV_LIBRARIES}
)

#add_executable(scenic_test test/test_scenic.cpp)
#target_link_libraries(scenic_test
#    GTest::GTest
#    GTest::Main
#    glog::glog
#    ${PROJECT_NAME}
#)

#add_executable(graph_test test/test_graph.cpp)
#target_link_libraries(graph_test
#    GTest::GTest
#    GTest::Main
#    ${PROJECT_NAME}
#)

#add_executable(kmeans_test examples/test_kmeans.cpp)
#target_link_libraries(kmeans_test
#    ${PROJECT_NAME}
#)

#include(GoogleTest)
#gtest_discover_tests(scenic_test)
#gtest_discover_tests(graph_test)

install(TARGETS ${PROJECT_NAME} glider stickyvo
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Generate and install CMake config files
include(CMakePackageConfigHelpers)

# Generate the config file that includes the exports
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

# Generate the version file
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION 1.0.0
  COMPATIBILITY AnyNewerVersion
)

# Install the config files
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# Install the export set
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)
