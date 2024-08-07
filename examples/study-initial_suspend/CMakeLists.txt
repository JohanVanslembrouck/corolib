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
    p0000_void

    p0100_void 
    p0110_bool
    p0120_coroutine_handle
  )
  add_executable(stis-${_target} ${_target}.cpp)
endforeach()

foreach(_target
    p1000_void
    p1000e_void
    p1000l_void

    p1010_bool
    p1010e_bool
    p1010l_bool

    p1020_coroutine_handle
    p1020e_coroutine_handle
    p1020l_coroutine_handle

    p1100_void
    p1100e_void
    p1100l_void

    p1110_bool
    p1110e_bool
    p1110l_bool

    p1120_coroutine_handle
    p1120e_coroutine_handle
    p1120l_coroutine_handle

    p1200e_void
    p1200l_void
    p1210e_bool
    p1210l_bool
    p1220e_coroutine_handle
    p1220l_coroutine_handle

    p1320_coroutine_handle 
    p1320e_coroutine_handle
    p1320l_coroutine_handle 
    p1320e_coroutine_handle-thread
    p1320l_coroutine_handle-thread

    p1322_coroutine_handle
    p1322e_coroutine_handle
    p1322l_coroutine_handle
    p1322e_coroutine_handle-thread
    p1322l_coroutine_handle-thread
  )
  add_executable(stis-${_target} ${_target}.cpp tracker1.cpp)
endforeach()

include_directories(../../include)

foreach(_target
    p1030_corolib
    p1030e_corolib

    p1130_corolib
    p1130e_corolib

    p1230_corolib
    p1230e_corolib

    p1330_corolib
    p1330_corolib-thread
    p1330e_corolib
    p1332e_corolib

    p1332_corolib
    p1332_corolib-thread
    p1330e_corolib-thread
    p1332e_corolib-thread
  )
  add_executable(stis-${_target} ${_target}.cpp tracker1.cpp)
  target_link_libraries(stis-${_target} corolib)
endforeach()
