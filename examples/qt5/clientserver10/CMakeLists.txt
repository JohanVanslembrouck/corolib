project (clientserver10)

include_directories(../../../include
                    ../clientserver11
                    ../common)

add_executable(qt-tcpclient00 tcpclientmain00.cpp tcpclient00.cpp ../clientserver11/tcpconfigfile.cpp ../clientserver11/tcpconfig.cpp)
target_link_libraries(qt-tcpclient00 corolib-qt5 corolib Qt5::Core Qt5::Network)
