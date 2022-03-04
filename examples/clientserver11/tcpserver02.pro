QT -= gui
QT += network

CONFIG += c++11 console
CONFIG -= app_bundle

QMAKE_CXXFLAGS += /await:strict

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        tcpconfig.cpp \
        tcpconfigfile.cpp \
        tcpserver02.cpp \
        tcpservermain02.cpp \
        ../common-qt/crc.cpp \
        ../common-qt/protocolmessage.cpp \
        ../common-qt/tcpserver.cpp \
        ../../lib/print.cpp \
        ../../lib/async_operation.cpp \

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
        ../../include/corolib/async_operation.h \
        ../../include/corolib/async_task.h \
        ../../include/corolib/commservice.h \
        ../../include/corolib/print.h \
        ../../include/corolib/wait_all_counter.h \
        ../../include/corolib/wait_any.h \
        ../common-qt/connectioninfo.h \
        tcpconfig.h \
        tcpconfigfile.h \
        connectioninfo.h \
        tcpserver02.h \
        ../common-qt/crc.h \
        ../common-qt/protocolmessage.h \
        ../common-qt/tcpserver.h

INCLUDEPATH += \
    ../common-qt \
    ../../include \
