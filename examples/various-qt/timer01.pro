QT -= gui
QT += network

CONFIG += c++11 console
CONFIG -= app_bundle

QMAKE_CXXFLAGS += /await

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        timer01.cpp \
        timermain01.cpp \
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
        ../../include/corolib/wait_all_awaitable.h \
        timer01.h \

INCLUDEPATH += \
    ../common-qt \
    ../../include/corolib \
