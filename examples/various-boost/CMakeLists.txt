cmake_minimum_required(VERSION 3.17)

project (timers)

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
	set(CMAKE_CXX_FLAGS "/await:strict /EHsc /D _WIN32_WINNT=0x0601")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_CXX_FLAGS "-Wall -std=c++20 -fcoroutines")
else()
	message("Unsupported compiler")
endif()

include_directories(../../include ${Boost_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIR})

add_executable(timer01 timermain01.cpp timer01.cpp)
target_link_libraries(timer01 corolib)
