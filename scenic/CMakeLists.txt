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
# my packages
find_package(Clipper REQUIRED)
find_package(glider REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
    ${yaml-cpp_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
    src/kmeans.cpp
    src/params.cpp
    src/segmentation_processor.cpp
    src/graphing_processor.cpp
    src/scenic.cpp
    src/node.cpp
    src/graph.cpp
    src/edge.cpp
    src/rectifier.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${OpenCV_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    yaml-cpp
    glog::glog
    Clipper::clipper
    glider::glider
)

add_executable(scenic_test test/test_scenic.cpp)
target_link_libraries(scenic_test
    GTest::GTest
    GTest::Main
    glog::glog
    ${PROJECT_NAME}
)

#add_executable(graph_test test/test_graph.cpp)
#target_link_libraries(graph_test
#    GTest::GTest
#    GTest::Main
#    ${PROJECT_NAME}
#)

add_executable(kmeans_test examples/test_kmeans.cpp)
target_link_libraries(kmeans_test
    ${PROJECT_NAME}
)

include(GoogleTest)
gtest_discover_tests(scenic_test)
#gtest_discover_tests(graph_test)

install(TARGETS ${PROJECT_NAME}
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
