cmake_minimum_required(VERSION 3.0)

set(BOSCOMM_VERSION_MAJOR 1)
set(BOSCOMM_VERSION_MINOR 4)

set(CMAKE_BUILD_TYPE Debug)

option(USE_TEST_MAIN "Enable Testing Mode" OFF)

project(hexabitz)

configure_file ( ${PROJECT_SOURCE_DIR}/Config.h.in ${PROJECT_SOURCE_DIR}/include/Config.h )

include_directories(include/ "${PROJECT_BINARY_DIR}")

set (CMAKE_CXX_STANDARD 17)

add_library(hexabitz SHARED src/hal/Serial.cpp src/helper/helper.cpp src/helper/BinaryStream.cpp 
                    src/helper/GenericStream.cpp src/hexabitz/BOSMessage.cpp
					src/hexabitz/ProxyModule.cpp src/hexabitz/BOSFrame.cpp 
                    src/hexabitz/BOSMessageBuilder.cpp src/hexabitz/BOS.cpp 
                    src/hexabitz/Service.cpp src/hexabitz/ModuleInfo.cpp)

# set_property(TARGET hexabitz PROPERTY CXX_STANDARD 17)


add_executable(demo src/tools/demo-H01R0.cpp)
target_link_libraries(demo hexabitz)