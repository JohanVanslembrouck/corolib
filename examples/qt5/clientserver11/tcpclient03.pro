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
        tcpclient03.cpp \
        tcpclientmain03.cpp \
        ../common/tcpclient.cpp \
        ../common/tcpclientco1.cpp

HEADERS += \
        ../../../include/corolib/async_operation.h \
        ../../../include/corolib/async_task.h \
        ../../../include/corolib/commservice.h \
        ../../../include/corolib/print.h \
        ../../../include/corolib/when_all_counter.h \
        ../../../include/corolib/when_any.h \
        tcpconfig.h \
        tcpconfigfile.h \
        tcpclient03.h \
        ../common/connectioninfo.h \
        ../common/crc.h \
        ../common/protocolmessage.h \
        ../common/tcpclientco1.h \
        ../common/tcpclient.h

INCLUDEPATH += \
    ../common \
    ../../../include

LIBS += -L../../../ -lcorolib -lcommonqt
