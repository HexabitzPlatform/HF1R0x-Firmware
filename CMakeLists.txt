cmake_minimum_required(VERSION 3.0)

set(BOSCOMM_VERSION_MAJOR 1)
set(BOSCOMM_VERSION_MINOR 0)

option(USE_TEST_MAIN "Enable Testing Mode" OFF)

project(hexabitz)

configure_file ( ${PROJECT_SOURCE_DIR}/Config.h.in ${PROJECT_SOURCE_DIR}/include/Config.h )

add_executable(boscomm src/hal/Serial.cpp)

include_directories(include/ "${PROJECT_BINARY_DIR}")
