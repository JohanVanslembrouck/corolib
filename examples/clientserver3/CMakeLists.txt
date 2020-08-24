cmake_minimum_required(VERSION 3.17)

project (clientserver3)

set(CMAKE_CXX_FLAGS "/await /EHsc /D _WIN32_WINNT=0x0601")

include_directories(../../include ${Boost_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIR})

add_executable(cs3-client1 client1.cpp)
add_executable(cs3-server server.cpp)
add_executable(cs3-server2 server2.cpp)
