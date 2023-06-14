project(coroboostlibrary)

include_directories(../../../include ${Boost_INCLUDE_DIR})

add_library(corolib-boost commcore.cpp commclient.cpp commserver.cpp)

target_link_libraries(corolib)

install(TARGETS corolib
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
