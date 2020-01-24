AM_CXXFLAGS = -std=c++17

lib_LTLIBRARIES = libhexabitz.la
libhexabitz_la_LDFLAGS = -shared
libhexabitz_la_LDFLAGS += -version-info 0:0:0
libhexabitz_la_CPPFLAGS = "-I$(top_srcdir)/include"
libhexabitz_la_SOURCES  = $(top_srcdir)/src/hal/Serial.cpp $(top_srcdir)/include/hal/Serial.h 
libhexabitz_la_SOURCES += $(top_srcdir)/src/helper/BinaryStream.cpp include/helper/BinaryStream.h
libhexabitz_la_SOURCES += $(top_srcdir)/src/helper/GenericStream.cpp include/helper/GenericStream.h $(top_srcdir)/src/helper/helper.cpp include/helper/helper.h
libhexabitz_la_SOURCES += $(top_srcdir)/src/hexabitz/BOS.cpp include/hexabitz/BOS.h $(top_srcdir)/src/hexabitz/BOSFrame.cpp include/hexabitz/BOSFrame.h
libhexabitz_la_SOURCES += $(top_srcdir)/src/hexabitz/BOSMessageBuilder.cpp include/hexabitz/BOSMessageBuilder.h
libhexabitz_la_SOURCES += $(top_srcdir)/src/hexabitz/BOSMessage.cpp include/hexabitz/BOSMessage.h
libhexabitz_la_SOURCES += $(top_srcdir)/src/hexabitz/ModuleInfo.cpp include/hexabitz/ModuleInfo.h
libhexabitz_la_SOURCES += $(top_srcdir)/src/hexabitz/ProxyModule.cpp include/hexabitz/ProxyModule.h
libhexabitz_la_SOURCES += $(top_srcdir)/src/hexabitz/Service.cpp include/hexabitz/Service.h