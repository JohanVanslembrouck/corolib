QT -= gui
QT += network

CONFIG += c++17 console
CONFIG -= app_bundle

QMAKE_CXXFLAGS += /await

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        tcpconfig.cpp \
        tcpconfigfile.cpp \
        tcpclient01.cpp \
        tcpclientmain01.cpp \
        ../common-qt/crc.cpp \
        ../common-qt/protocolmessage.cpp \
        ../common-qt/tcpclient.cpp \
        ../../lib/print.cpp \

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
        tcpconfig.h \
        tcpconfigfile.h \
        tcpclient01.h \
        ../common-qt/connectioninfo.h \
        ../common-qt/crc.h \
        ../common-qt/protocolmessage.h \
        ../common-qt/tcpclient.h

INCLUDEPATH += \
    ../common-qt \
    ../../include \
