cmake_minimum_required(VERSION 3.14)

project (corolib)

# If you have installed Boost and you want to use it, set the following variable to TRUE:
set(BOOST_INSTALLED FALSE)
# If you have installed gRPC and you want to use it, set the following variable to TRUE:
set(GRPC_INSTALLED FALSE)
# If you have installed TAO and you want to use it, set the following variable to TRUE:
set(TAO_INSTALLED FALSE)
# If you have installed Qt5 and you want to use it, set the following variable to TRUE:
set(QT5_INSTALLED FALSE)
# If you have installed libevent and you want to use it, set the following variable to TRUE:
set(LIBEVENT_INSTALLED FALSE)
# If you have installed curl and you want to use it, set the following variable to TRUE:
set(LIBCURL_INSTALLED FALSE)

# If you want to install and use GoogleTest, set the following variable to TRUE:
set(USE_GOOGLETEST TRUE)

# To build the studies, set the following variable to TRUE:
set(BUILD_STUDIES FALSE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# USE_IN_MT_APPS can be set to 0 to build corolib for single-threaded frameworks and applications.

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CMAKE_CXX_FLAGS "/await:strict /EHsc /D USE_CPP20SEMAPHORE=1 /D USE_IN_MT_APPS=1")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    # -lstdc++ had to be added to build with gcc 13.3.0 on Ubuntu 24.04.
    # This link option is not necessary to build with gcc 11.4.0 on Ubuntu 22.04.
    set(CMAKE_CXX_FLAGS "-Wall -fPIC -fcoroutines -D USE_CPP20SEMAPHORE=1 -D USE_IN_MT_APPS=1 -lstdc++")
else()
    message("Unsupported compiler")
endif()

add_subdirectory(lib)

if (BOOST_INSTALLED)
    if (WIN32)
        # Please adapt this to your local Boost installation
        set(BOOST_ROOT C:/local/boost/boost_1_82_0/)
    endif()
    add_subdirectory(examples/boost)
endif()

if (GRPC_INSTALLED)
    add_subdirectory(examples/grpc)
endif()

if (TAO_INSTALLED)
    add_subdirectory(examples/tao/Buffered_AMI)
endif()

if (QT5_INSTALLED)
    add_subdirectory(examples/qt5)
endif()

if (LIBEVENT_INSTALLED)
    add_subdirectory(examples/libevent)
endif()

if (LIBCURL_INSTALLED)
    add_subdirectory(examples/curl)
endif()

if (WIN32)
    add_subdirectory(examples/cppcoro)
endif()

add_subdirectory(examples/tutorial)

if (BUILD_STUDIES)
    add_subdirectory(studies)
endif()

if (USE_GOOGLETEST)
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

    add_subdirectory(tests)
endif()
