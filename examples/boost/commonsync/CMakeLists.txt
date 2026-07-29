project(coroboostlibrarysync)

include_directories(../../../include)

add_library(corolib-boostsync runeventqueue.cpp commcoresync.cpp commclientsync.cpp)

target_link_libraries(corolib)

install(TARGETS corolib
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

project(coroboostlibrarylso)
