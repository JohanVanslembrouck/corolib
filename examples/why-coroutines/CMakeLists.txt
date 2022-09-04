
project (tutorial)

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
	set(CMAKE_CXX_FLAGS "/await:strict /EHsc /D _WIN32_WINNT=0x0601")
else()
	set(CMAKE_CXX_FLAGS "-Wall")
endif()

include_directories(../../include)

add_executable(co-less01 co-less01.cpp)
add_executable(co-less01th co-less01th.cpp)
add_executable(co-less02 co-less02.cpp)
add_executable(co-less03 co-less03.cpp)
add_executable(co-less04 co-less04.cpp)
add_executable(co-less05 co-less05.cpp)
add_executable(co-less06 co-less06.cpp)
add_executable(co-less07 co-less07.cpp)

add_executable(co-full01 co-full01.cpp)
target_link_libraries(co-full01 corolib)

add_executable(co-full02 co-full02.cpp)
target_link_libraries(co-full02 corolib)

add_executable(co-full03 co-full03.cpp)
target_link_libraries(co-full03 corolib)
