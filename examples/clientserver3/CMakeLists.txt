cmake_minimum_required(VERSION 3.17)

project (clientserver3)

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
	set(CMAKE_CXX_FLAGS "/await /EHsc /D _WIN32_WINNT=0x0601")
else()
	set(CMAKE_CXX_FLAGS "-Wall")
endif()

include_directories(../../include ${Boost_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIR})

set(LibDir "../../lib")

add_executable(cs3-client1 client1.cpp ${LibDir}/print.cpp)
add_executable(cs3-client2 client2.cpp ${LibDir}/print.cpp)

add_executable(cs3-server server.cpp ${LibDir}/print.cpp)
add_executable(cs3-server2 server2.cpp ${LibDir}/print.cpp)
add_executable(cs3-server3 server3.cpp ${LibDir}/print.cpp)
add_executable(cs3-server4 server4.cpp ${LibDir}/print.cpp)
add_executable(cs3-server5 server5.cpp ${LibDir}/print.cpp)
