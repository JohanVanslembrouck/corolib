cmake_minimum_required(VERSION 3.17)

project (clientserver1)

set(CMAKE_CXX_FLAGS "/await /EHsc /D _WIN32_WINNT=0x0601")

include_directories(../../include ${Boost_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIR})

add_executable(client1 client1.cpp)
target_link_libraries(client1)

add_executable(client3 client3.cpp)
target_link_libraries(client3)

add_executable(client3WA client3WA.cpp)
target_link_libraries(client3WA)

add_executable(clientserver clientserver.cpp)
target_link_libraries(clientserver)

add_executable(server server.cpp)
target_link_libraries(server)
