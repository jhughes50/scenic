cmake_minimum_required(VERSION 3.0.2)
project(scenic)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(Json_INCLUDE_DIRS "/usr/include/jsoncpp")

# apt installable packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)
find_package(yaml-cpp REQUIRED)
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

add_library(${PROJECT_NAME}
    src/kmeans.cpp
    src/segmentation_processor.cpp
    src/scenic.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${OpenCV_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    yaml-cpp
    Clipper::clipper
    glider::glider
)
