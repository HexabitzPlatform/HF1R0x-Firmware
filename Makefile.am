bin_PROGRAMS = boscomm
boscomm_CPPFLAGS = "-I../include"
AM_CXXFLAGS = -std=c++17
boscomm_SOURCES  = src/hal/Serial.cpp include/hal/Serial.h 
boscomm_SOURCES += src/helper/BinaryStream.cpp include/helper/BinaryStream.h
boscomm_SOURCES += src/helper/GenericStream.cpp include/helper/GenericStream.h src/helper/helper.cpp include/helper/helper.h
boscomm_SOURCES += src/hexabitz/BOS.cpp include/hexabitz/BOS.h src/hexabitz/BOSFrame.cpp include/hexabitz/BOSFrame.h
boscomm_SOURCES += src/hexabitz/BOSMessageBuilder.cpp include/hexabitz/BOSMessageBuilder.h
boscomm_SOURCES += src/hexabitz/BOSMessage.cpp include/hexabitz/BOSMessage.h
boscomm_SOURCES += src/hexabitz/ModuleInfo.cpp include/hexabitz/ModuleInfo.h
boscomm_SOURCES += src/hexabitz/ProxyModule.cpp include/hexabitz/ProxyModule.h
boscomm_SOURCES += src/hexabitz/Service.cpp include/hexabitz/Service.h
boscomm_SOURCES += src/main/main.cpp


lib_LTLIBRARIES = libhexabitz.la
libhexabitz_la_LDFLAGS = -shared
libhexabitz_la_CPPFLAGS = "-I../include"
libhexabitz_la_SOURCES  = src/hal/Serial.cpp include/hal/Serial.h 
libhexabitz_la_SOURCES += src/helper/BinaryStream.cpp include/helper/BinaryStream.h
libhexabitz_la_SOURCES += src/helper/GenericStream.cpp include/helper/GenericStream.h src/helper/helper.cpp include/helper/helper.h
libhexabitz_la_SOURCES += src/hexabitz/BOS.cpp include/hexabitz/BOS.h src/hexabitz/BOSFrame.cpp include/hexabitz/BOSFrame.h
libhexabitz_la_SOURCES += src/hexabitz/BOSMessageBuilder.cpp include/hexabitz/BOSMessageBuilder.h
libhexabitz_la_SOURCES += src/hexabitz/BOSMessage.cpp include/hexabitz/BOSMessage.h
libhexabitz_la_SOURCES += src/hexabitz/ModuleInfo.cpp include/hexabitz/ModuleInfo.h
libhexabitz_la_SOURCES += src/hexabitz/ProxyModule.cpp include/hexabitz/ProxyModule.h
libhexabitz_la_SOURCES += src/hexabitz/Service.cpp include/hexabitz/Service.h