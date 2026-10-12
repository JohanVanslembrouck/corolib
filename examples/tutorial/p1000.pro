TEMPLATE = app

QT -= gui

CONFIG += c++20 console
CONFIG -= app_bundle

include(../../qt5.pri)

SOURCES += \
        p1000.cpp

HEADERS += \
        ../../include/corolib/async_task.h \
        ../../include/corolib/print.h \

INCLUDEPATH += \
    ../../include

LIBS += -L../../ -lcorolib
