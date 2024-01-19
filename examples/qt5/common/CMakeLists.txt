project(coroqt5library)

include_directories(../../../include)

add_library(corolib-qt5 crc.cpp protocolmessage.cpp tcpclient.cpp tcpclientco.cpp tcpclientco1.cpp tcpserver.cpp)

target_link_libraries(corolib-qt5)

install(TARGETS corolib-qt5
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
