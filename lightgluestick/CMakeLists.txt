cmake_minimum_required(VERSION 3.16)
project(lgs LANGUAGES CXX)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

include_directories(
    include
)

add_library(lgs SHARED
  src/lgs_frontend.cpp
)

target_link_libraries(lgs PRIVATE
  pybind11::embed
  Python3::Python
)

# Get Python site-packages directory
execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import site; print(site.getsitepackages()[0])"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

message(STATUS "Python site-packages: ${PYTHON_SITE_PACKAGES}")

# Installation rules
install(TARGETS lgs
    EXPORT lgsTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib 
    RUNTIME DESTINATION bin 
)

# Install headers
install(DIRECTORY include/
    DESTINATION include 
)

# Install Python package - THIS IS THE KEY ADDITION
install(DIRECTORY python/
    DESTINATION ${PYTHON_SITE_PACKAGES}/stickyvo_lgs
    FILES_MATCHING PATTERN "*.py"
    PATTERN "__pycache__" EXCLUDE
)

# Export targets
install(EXPORT lgsTargets
    FILE lgsTargets.cmake
    NAMESPACE lgs::
    DESTINATION lib/cmake/lgs
)

# Config file setup
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/lgsConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/lgsConfig.cmake
    INSTALL_DESTINATION lib/cmake/lgs
    PATH_VARS PYTHON_SITE_PACKAGES
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/lgsConfig.cmake
    DESTINATION lib/cmake/lgs
)
