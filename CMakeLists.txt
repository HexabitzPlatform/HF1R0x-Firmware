cmake_minimum_required(VERSION 3.3)


include(GNUInstallDirs)

set(HEXABITZ_VERSION_MAJOR 1)
set(HEXABITZ_VERSION_MINOR 5)

# C & C++
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
option(USE_TEST_MAIN "Enable Testing Mode" OFF)

project(hexabitz VERSION "${HEXABITZ_VERSION_MAJOR}.${HEXABITZ_VERSION_MINOR}" 
                DESCRIPTION "Hexabitz Open-Source Libraries to interface modules with Linux")

configure_file(${CMAKE_SOURCE_DIR}/Config.h.in ${PROJECT_SOURCE_DIR}/include/Config.h)

############################## Shared Library ##############################
add_library(hexabitz SHARED src/hal/Serial.cpp src/helper/helper.cpp 
                    src/helper/BinaryStream.cpp 
                    src/helper/GenericStream.cpp src/hexabitz/BOSMessage.cpp
					src/hexabitz/ProxyModule.cpp src/hexabitz/BOSFrame.cpp 
                    src/hexabitz/BOSMessageBuilder.cpp src/hexabitz/BOS.cpp 
                    src/hexabitz/Service.cpp src/hexabitz/ModuleInfo.cpp)

target_include_directories(hexabitz PUBLIC include/)
set_target_properties(hexabitz PROPERTIES VERSION ${PROJECT_VERSION})
set_property(TARGET hexabitz PROPERTY CXX_STANDARD 17)

set(PUBLIC_INCLUDE_FILES 
            include/hal/Serial.h include/helper/helper.h 
            include/helper/BinaryStream.h 
            include/helper/GenericStream.h include/hexabitz/BOSMessage.h
            include/hexabitz/ProxyModule.h include/hexabitz/BOSFrame.h 
            include/hexabitz/BOSMessageBuilder.h include/hexabitz/BOS.h 
            include/hexabitz/Service.h include/hexabitz/ModuleInfo.h
)

# set_target_properties(hexabitz PROPERTIES PUBLIC_HEADER "${PUBLIC_INCLUDE_FILES}")

install(TARGETS hexabitz
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)

install(DIRECTORY "include/" DESTINATION include/hexabitz)

############################# Demo Application #############################
add_executable(hexabitz-demo src/tools/demo-H01R0.cpp)
target_link_libraries(hexabitz-demo hexabitz)
set_property(TARGET hexabitz-demo PROPERTY CXX_STANDARD 17)

install(TARGETS hexabitz-demo RUNTIME DESTINATION bin)

# Cpack
set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "abhiarora")
include(CPack)
