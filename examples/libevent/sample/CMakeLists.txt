project (libevent-sample)

if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    # Please check these paths with your installation:
    include_directories(C:/local/libevent/include 
                        C:/local/libevent/build/include)
    link_directories(C:/local/libevent/build/lib/Release)

    set(EXTRALIBS Ws2_32 Ws2_32)
endif()

add_executable(hello-world-server hello-world-server.cpp)
target_link_libraries(hello-world-server event_extra event_core ${EXTRALIBS})

add_executable(hello-world-client hello-world-client.cpp)
target_link_libraries(hello-world-client event_extra event_core ${EXTRALIBS})

add_executable(hello-world-client2 hello-world-client2.cpp)
target_link_libraries(hello-world-client2 event_extra event_core corolib ${EXTRALIBS})

include_directories(../../../include)

add_executable(hello-world-coroutine-client2 hello-world-coroutine-client2.cpp)
target_link_libraries(hello-world-coroutine-client2 event_extra event_core corolib ${EXTRALIBS})
