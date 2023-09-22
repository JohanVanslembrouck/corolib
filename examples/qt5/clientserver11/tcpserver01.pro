TEMPLATE = app

QT -= gui
QT += network

CONFIG += c++11 console
CONFIG -= app_bundle

SOURCES += \
        tcpconfig.cpp \
        tcpconfigfile.cpp \
        tcpserver01.cpp \
        tcpservermain01.cpp

HEADERS += \
        tcpconfig.h \
        tcpconfigfile.h \
        connectioninfo.h \
        tcpserver01.h \
        ../common/connectioninfo.h \
        ../common/crc.h \
        ../common/protocolmessage.h \
        ../common/tcpserver.h

INCLUDEPATH += \
    ../common

LIBS += -L../../../ -lcommonqt
