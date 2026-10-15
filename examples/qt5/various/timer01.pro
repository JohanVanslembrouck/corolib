TEMPLATE = app

QT -= gui
QT += network

CONFIG += c++20 console
CONFIG -= app_bundle

include(../../../qt5.pri)

SOURCES += \
        timer01.cpp \
        timermain01.cpp

HEADERS += \
        ../../../include/corolib/async_operation.h \
        ../../../include/corolib/async_task.h \
        ../../../include/corolib/commservice.h \
        ../../../include/corolib/print.h \
        ../../../include/corolib/when_all.h \
        timer01.h

INCLUDEPATH += \
    ../../../include

LIBS += -L../../../ -lcorolib

TARGET = qt-timer01
