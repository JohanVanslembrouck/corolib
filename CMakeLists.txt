cmake_minimum_required(VERSION 3.14)

project (corolib)

if (WIN32)
    set(Boost_INCLUDE_DIR C:/local/boost/boost_1_82_0)
    set(Boost_LIBRARY_DIR C:/local/boost/boost_1_82_0/stage/lib)
elseif (UNIX)
    set(Boost_INCLUDE_DIR /usr/include/boost)
    set(Boost_LIBRARY_DIR /usr/lib/x86_64-linux-gnu/)
else()
    message("Unsupported operating system")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CMAKE_CXX_FLAGS "/await:strict /EHsc /D _WIN32_WINNT=0x0601")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_CXX_FLAGS "-Wall -fcoroutines")
else()
    message("Unsupported compiler")
endif()

add_subdirectory(lib)

add_subdirectory(examples/boost/common)
add_subdirectory(examples/boost/clientserver1)
add_subdirectory(examples/boost/clientserver2)
add_subdirectory(examples/boost/clientserver3)
add_subdirectory(examples/boost/clientserver4)
add_subdirectory(examples/boost/various)

add_subdirectory(examples/tutorial)
add_subdirectory(examples/tutorial/test)

add_subdirectory(examples/why-coroutines)

add_subdirectory(examples/corolab)

add_subdirectory(examples/grpc/cpp/helloworld)
add_subdirectory(examples/grpc/cpp/route_guide)

if (NOT googletest)
	include(FetchContent)
	FetchContent_Declare(
		googletest
		# Specify the commit you depend on and update it regularly.
		URL https://github.com/google/googletest/archive/refs/heads/master.zip
	)
	# For Windows: Prevent overriding the parent project's compiler/linker settings
	set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
	FetchContent_MakeAvailable(googletest)
endif()
