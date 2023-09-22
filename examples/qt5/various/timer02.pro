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
        timer02.cpp \
        timermain02.cpp

HEADERS += \
        ../../../include/corolib/async_operation.h \
        ../../../include/corolib/async_task.h \
        ../../../include/corolib/commservice.h \
        ../../../include/corolib/print.h \
        ../../../include/corolib/when_all.h \
        timer02.h

INCLUDEPATH += \
    ../../../include

LIBS += -L../../../ -lcorolib
