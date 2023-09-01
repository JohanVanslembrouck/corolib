TEMPLATE = lib

QT -= gui
QT += network

CONFIG += plugin static

win32 {
QMAKE_CXXFLAGS += /await:strict
}
unix {
QMAKE_CXXFLAGS += -fcoroutines
}

INCLUDEPATH += \
    ../../../include

HEADERS *= connectioninfo.h \
        crc.h \
        protocolmessage.h \
        tcpclient.h \
        tcpclientco.h \
        tcpclientco1.h \
        tcpserver.h
				
SOURCES += crc.cpp \
        protocolmessage.cpp \
        tcpclient.cpp \
        tcpclientco.cpp \
        tcpclientco1.cpp \
        tcpserver.cpp

TARGET = commonqt

DESTDIR = ../../../
