TEMPLATE = app

QT -= gui

CONFIG += c++20 console
CONFIG -= app_bundle

win32 {
QMAKE_CXXFLAGS += /await:strict
}
unix {
QMAKE_CXXFLAGS += -fcoroutines
}

SOURCES += \
        p1000.cpp

HEADERS += \
        ../../include/corolib/async_task.h \
        ../../include/corolib/print.h \

INCLUDEPATH += \
    ../../include

LIBS += -L../../ -lcorolib
