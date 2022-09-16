cmake_minimum_required(VERSION 3.17)

project (clientserver2)

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
	set(CMAKE_CXX_FLAGS "/await:strict /EHsc /D _WIN32_WINNT=0x0601")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_CXX_FLAGS "-Wall -std=c++20 -fcoroutines")
else()
	message("Unsupported compiler")
endif()

include_directories(../../include ${Boost_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIR})

add_executable(cs2-client1 client1.cpp)
target_link_libraries(cs2-client1 corolib corolib-boost)
add_executable(cs2-client1a client1a.cpp)
target_link_libraries(cs2-client1a corolib corolib-boost)
add_executable(cs2-server server.cpp)
target_link_libraries(cs2-server corolib corolib-boost)
add_executable(cs2-server2 server2.cpp)
target_link_libraries(cs2-server2 corolib corolib-boost)
