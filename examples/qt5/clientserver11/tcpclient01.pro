TEMPLATE = app

QT -= gui
QT += network

CONFIG += c++20 console
CONFIG -= app_bundle

win32 {
QMAKE_CXXFLAGS += /await:strict
}
unix {
QMAKE_CXXFLAGS += -fcoroutines
}

SOURCES += \
        tcpconfig.cpp \
        tcpconfigfile.cpp \
        tcpclient01.cpp \
        tcpclientmain01.cpp \
        ../common/tcpclient.cpp

HEADERS += \
        ../../../include/corolib/async_operation.h \
        ../../../include/corolib/async_task.h \
        ../../../include/corolib/commservice.h \
        ../../../include/corolib/print.h \
        ../../../include/corolib/when_all_counter.h \
        ../../../include/corolib/when_any.h \
        tcpconfig.h \
        tcpconfigfile.h \
        tcpclient01.h \
        ../common/connectioninfo.h \
        ../common/crc.h \
        ../common/protocolmessage.h \
        ../common/tcpclient.h

INCLUDEPATH += \
    ../common \
    ../../../include

LIBS += -L../../../ -lcorolib -lcommonqt

TARGET = qt-tcpclient01
