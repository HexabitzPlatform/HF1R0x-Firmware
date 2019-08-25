cmake_minimum_required(VERSION 3.0)

set(BOSCOMM_VERSION_MAJOR 1)
set(BOSCOMM_VERSION_MINOR 0)

option(USE_TEST_MAIN "Enable Testing Mode" OFF)

project(hexabitz)

configure_file ( ${PROJECT_SOURCE_DIR}/Config.h.in ${PROJECT_SOURCE_DIR}/include/Config.h )


add_executable(boscomm src/hal/Serial.cpp src/helper/helper.cpp src/helper/BinaryStream.cpp src/helper/GenericStream.cpp src/hexabitz/BOSMessage.cpp
						src/hexabitz/Module.cpp src/hexabitz/BOSFrame.cpp src/hexabitz/BOSMessageBuilder.cpp src/main/main.cpp)

include_directories(include/ "${PROJECT_BINARY_DIR}")

set_property(TARGET boscomm PROPERTY CXX_STANDARD 17)
