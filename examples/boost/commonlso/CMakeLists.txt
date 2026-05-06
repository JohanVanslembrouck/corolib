project(coroboostlibrarylso)

include_directories(../../../include)

add_library(corolib-boostlso
            accept_operation.cpp
            connect_operation.cpp
            read_operation.cpp
            write_operation.cpp
            timing_operation.cpp
            commcorelso.cpp
            commclientlso.cpp
            commserverlso.cpp)

target_link_libraries(corolib)

install(TARGETS corolib
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
