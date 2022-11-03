
project(corolibrary)

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
	set(CMAKE_CXX_FLAGS "/await:strict /EHsc /D _WIN32_WINNT=0x0601")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_CXX_FLAGS "-Wall -std=c++20 -fcoroutines")
else()
	message("Unsupported compiler")
endif()

include_directories(../include ${Boost_INCLUDE_DIR})

add_library(corolib print.cpp async_operation.cpp commservice.cpp )
add_library(corolib-boost commcore.cpp commclient.cpp commserver.cpp)

target_link_libraries(corolib)

install(TARGETS corolib
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
