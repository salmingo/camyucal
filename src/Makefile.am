bin_PROGRAMS=camyucal
camyucal_SOURCES=IOServiceKeep.cpp udpasio.cpp CameraBase.cpp CameraGYCCD.cpp camyucal.cpp

camyucal_LDFLAGS=-L/usr/local/lib
BOOST_LIBS = -lboost_system-mt -lboost_thread-mt
camyucal_LDADD = ${BOOST_LIBS} -lm -lpthread
