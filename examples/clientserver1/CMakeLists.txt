cmake_minimum_required(VERSION 3.17)

project (clientserver1)

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
	set(CMAKE_CXX_FLAGS "/await /EHsc /D _WIN32_WINNT=0x0601")
else()
	set(CMAKE_CXX_FLAGS "-Wall")
endif()

include_directories(../../include ${Boost_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIR})

add_executable(cs1-client1 client1.cpp)
add_executable(cs1-client1a client1a.cpp)
add_executable(cs1-client1b client1b.cpp)
add_executable(cs1-client1c client1c.cpp)

add_executable(cs1-client3 client3.cpp)
add_executable(cs1-client3WA client3WA.cpp)
add_executable(cs1-client3WAny client3WAny.cpp)
add_executable(cs1-clientserver clientserver.cpp)
add_executable(cs1-server server.cpp)
