TEMPLATE = app

QT += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++20
CONFIG += console

win32 {
QMAKE_CXXFLAGS += /await:strict
}
unix {
QMAKE_CXXFLAGS += -fcoroutines
}

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    ../clientserver11/tcpconfig.cpp \
    ../clientserver11/tcpconfigfile.cpp \
    main.cpp \
    mainwindow.cpp \
    tcpclient01.cpp

HEADERS += \
    ../../../include/corolib/async_operation.h \
    ../../../include/corolib/async_task.h \
    ../../../include/corolib/commservice.h \
    ../../../include/corolib/print.h \
    ../../../include/corolib/when_all_counter.h \
    ../../../include/corolib/when_all.h \
    ../clientserver11/tcpconfig.h \
    ../clientserver11/tcpconfigfile.h \
    mainwindow.h \
    tcpclient01.h \
    ../common/connectioninfo.h \
    ../common/crc.h \
    ../common/protocolmessage.h \
    ../common/tcpclient.h

FORMS += \
    mainwindow.ui

INCLUDEPATH += \
    ../common \
    ../clientserver11 \
    ../../../include

LIBS += -L../../../ -lcorolib -lcommonqt

TARGET = qt-tcpclient01gui
