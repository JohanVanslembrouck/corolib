project (clientserver11)

include_directories(../../../include
                    ../common)

add_executable(qt-tcpclient01 tcpclientmain01.cpp tcpclient01.cpp tcpconfigfile.cpp tcpconfig.cpp)
target_link_libraries(qt-tcpclient01 corolib-qt5 corolib Qt5::Core Qt5::Network)

add_executable(qt-tcpclient02 tcpclientmain02.cpp tcpclient02.cpp tcpconfigfile.cpp tcpconfig.cpp)
target_link_libraries(qt-tcpclient02 corolib-qt5 corolib Qt5::Core Qt5::Network)

add_executable(qt-tcpclient03 tcpclientmain03.cpp tcpclient03.cpp tcpconfigfile.cpp tcpconfig.cpp)
target_link_libraries(qt-tcpclient03 corolib-qt5 corolib Qt5::Core Qt5::Network)

add_executable(qt-tcpserver01 tcpservermain01.cpp tcpserver01.cpp tcpconfigfile.cpp tcpconfig.cpp)
target_link_libraries(qt-tcpserver01 corolib-qt5 corolib Qt5::Core Qt5::Network)

add_executable(qt-tcpserver02 tcpservermain02.cpp tcpserver02.cpp tcpconfigfile.cpp tcpconfig.cpp)
target_link_libraries(qt-tcpserver02 corolib-qt5 corolib Qt5::Core Qt5::Network)
