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
        rvo2.cpp
