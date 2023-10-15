cmake_minimum_required(VERSION 3.14)

project (corolib)

set(BOOST_INSTALLED TRUE)
set(GRPC_INSTALLED TRUE)
set(TAO_INSTALLED TRUE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CMAKE_CXX_FLAGS "/await:strict /EHsc")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_CXX_FLAGS "-Wall -fcoroutines")
else()
    message("Unsupported compiler")
endif()

add_subdirectory(lib)

if (BOOST_INSTALLED)
    add_subdirectory(examples/boost)
endif()

if (GRPC_INSTALLED)
    add_subdirectory(examples/grpc)
endif()

if (TAO_INSTALLED)
    add_subdirectory(examples/tao/Buffered_AMI)
endif()

if (WIN32)
    add_subdirectory(examples/cppcoro)
endif()

add_subdirectory(examples/tutorial)
add_subdirectory(examples/tutorial/test)

add_subdirectory(examples/why-coroutines)

add_subdirectory(examples/corolab)

add_subdirectory(examples/transform)

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
