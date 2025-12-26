cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(study_initial_suspend)

set (CMAKE_CXX_STANDARD 20)
set (CMAKE_CXX_EXTENSIONS OFF)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CMAKE_CXX_FLAGS "/await:strict /EHsc")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
set (MY_CXX_FLAGS 
    "-Wall \
    -Wextra \
    -Werror \
    -Wnon-virtual-dtor \
    -Wshadow \
    -Wsuggest-override \
    -pedantic \
    -pedantic-errors"
    )
endif()

foreach(_target
    p2010e_void-sc
    p2010l_void-sc
    p2020e_void-ma
    p2020l_void-ma
    p1100e_void
    p1100l_void
 )
 add_executable(cf-${_target} ${_target}.cpp print.cpp tracker.cpp counter.cpp)
endforeach()
