project (clientserver4)

include_directories(../../../include ../common ${Boost_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIR})

add_executable(cs4-client1 client1.cpp)
target_link_libraries(cs4-client1 corolib corolib-boost)

add_executable(cs4-server server.cpp)
target_link_libraries(cs4-server corolib corolib-boost)
add_executable(cs4-server2 server2.cpp)
target_link_libraries(cs4-server2 corolib corolib-boost)
