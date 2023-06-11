project (clientserver1)

include_directories(../../include ${Boost_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIR})

add_executable(cs1-client1 client1.cpp)
target_link_libraries(cs1-client1 corolib corolib-boost)
add_executable(cs1-client1a client1a.cpp)
target_link_libraries(cs1-client1a corolib corolib-boost)
add_executable(cs1-client1b client1b.cpp) 
target_link_libraries(cs1-client1b corolib corolib-boost)

add_executable(cs1-client1c client1c.cpp)
target_link_libraries(cs1-client1c corolib corolib-boost)

add_executable(cs1-client3 client3.cpp)
target_link_libraries(cs1-client3 corolib corolib-boost)
add_executable(cs1-client3WA client3WA.cpp)
target_link_libraries(cs1-client3WA corolib corolib-boost)
add_executable(cs1-client3WAny client3WAny.cpp)
target_link_libraries(cs1-client3WAny corolib corolib-boost)

add_executable(cs1-client4obs client4obs.cpp)
target_link_libraries(cs1-client4obs corolib corolib-boost)
add_executable(cs1-client4obs2 client4obs2.cpp)
target_link_libraries(cs1-client4obs2 corolib corolib-boost)
add_executable(cs1-client4obs3 client4obs3.cpp)
target_link_libraries(cs1-client4obs3 corolib corolib-boost)

add_executable(cs1-client5thr client5thr.cpp)
target_link_libraries(cs1-client5thr corolib corolib-boost)

add_executable(cs1-clientserver clientserver.cpp)
target_link_libraries(cs1-clientserver corolib corolib-boost)

add_executable(cs1-server server.cpp)
target_link_libraries(cs1-server corolib corolib-boost)

