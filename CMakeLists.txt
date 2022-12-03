# running script file to install dependencies
# install(CODE "execute_process(COMMAND install_dependencies.sh)")

cmake_minimum_required(VERSION 3.2.1)
project (kamikaze)

# Add project cmake modules to path.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Add opencv to the required libraries
# FIND_PACKAGE(OpenCV REQUIRED)
# INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS})
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# We probably don't want this to run on every build.
option(COVERAGE "Generate Coverage Data" OFF)

if (COVERAGE)
    include(CodeCoverage)
    set(LCOV_REMOVE_EXTRA "'vendor/*'")
    setup_target_for_coverage(code_coverage test/cpp-test coverage)
    set(COVERAGE_SRCS app/main.cpp include/lib.hpp)

    SET(CMAKE_CXX_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")
    SET(CMAKE_C_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage")
    SET(CMAKE_EXE_LINKER_FLAGS "-fprofile-arcs -ftest-coverage")
else()
    set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic -g")
endif()

include(CMakeToolsHelpers OPTIONAL)
set(CMAKE_CXX_STANDARD 14)

add_subdirectory(app)
add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(vendor/googletest/googletest)
