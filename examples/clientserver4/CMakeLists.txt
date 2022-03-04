cmake_minimum_required(VERSION 3.17)

project (clientserver4)

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
	set(CMAKE_CXX_FLAGS "/await:strict /EHsc /D _WIN32_WINNT=0x0601")
else()
	set(CMAKE_CXX_FLAGS "-Wall")
endif()

include_directories(../../include ${Boost_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIR})

add_executable(cs4-client1 client1.cpp)
target_link_libraries(cs4-client1 corolib)

add_executable(cs4-server server.cpp)
target_link_libraries(cs4-server corolib)
add_executable(cs4-server2 server2.cpp)
target_link_libraries(cs4-server2 corolib)
