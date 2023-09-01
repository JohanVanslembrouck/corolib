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
    ../include
				
SOURCES += async_operation.cpp \
        commservice.cpp \
        print.cpp

TARGET = corolib

DESTDIR = ../
