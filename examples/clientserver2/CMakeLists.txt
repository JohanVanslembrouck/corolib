cmake_minimum_required(VERSION 3.17)

project (clientserver2)

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
	set(CMAKE_CXX_FLAGS "/await /EHsc /D _WIN32_WINNT=0x0601")
else()
	set(CMAKE_CXX_FLAGS "-Wall")
endif()

include_directories(../../include ${Boost_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIR})

add_executable(cs2-client1 client1.cpp)
add_executable(cs2-client1a client1a.cpp)
add_executable(cs2-server server.cpp)
add_executable(cs2-server2 server2.cpp)
