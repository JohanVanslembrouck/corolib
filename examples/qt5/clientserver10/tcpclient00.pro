TEMPLATE = app

QT -= gui
QT += network

CONFIG += c++20 console
CONFIG -= app_bundle

SOURCES += \
        tcpclient00.cpp \
        tcpclientmain00.cpp \
        ../clientserver11/tcpconfig.cpp \
        ../clientserver11/tcpconfigfile.cpp \
        ../common/tcpclient.cpp

HEADERS += \
        tcpclient00.h \
        ../clientserver11/tcpconfig.h \
        ../clientserver11/tcpconfigfile.h \
        ../common/connectioninfo.h \
        ../common/crc.h \
        ../common/protocolmessage.h \
        ../common/tcpclient.h

INCLUDEPATH += \
    ../common \
    ../clientserver11 \
    ../../../include

LIBS += -L../../../ -lcommonqt

TARGET = qt-tcpclient00
