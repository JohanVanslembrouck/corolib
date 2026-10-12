TEMPLATE = lib

QT -= gui
QT += network

CONFIG += plugin static

include(../qt5.pri)

INCLUDEPATH += \
    ../include
				
SOURCES += async_operation.cpp \
        async_operation_register.cpp \
        commservice.cpp \
        eventqueue.cpp \
        tracker.cpp \
        print.cpp

TARGET = corolib

DESTDIR = ../
