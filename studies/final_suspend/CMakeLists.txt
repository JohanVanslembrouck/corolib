cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(study_final_suspend)

set (CMAKE_CXX_STANDARD 20)
set (CMAKE_CXX_EXTENSIONS OFF)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DUSE_CORO_DONE_TEST=0)

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
    p1000_sa
    p1000e_sa
    p1010_sa
    p1010e_sa
    p1020_sa
    p1020e_sa

    p1100_sn
    p1100e_sn
    p1110_sn
    p1110e_sn
    p1120_sn
    p1120e_sn

    p1150_sn2
    p1150e_sn2
    p1160_sn2
    p1160e_sn2
    p1170_sn2
    p1170e_sn2

    p1200_void
    p1200e_void
    p1210_void
    p1210e_void
    p1220_void
    p1220e_void

    p1300_bool
    p1300e_bool
    p1310_bool
    p1310e_bool
    p1320_bool
    p1320e_bool

    p1400_coroutine_handle
    p1400e_coroutine_handle
    p1410_coroutine_handle
    p1410e_coroutine_handle
    p1420_coroutine_handle
    p1420e_coroutine_handle
  )
  add_executable(stfs-${_target} ${_target}.cpp tracker.cpp print.cpp)
  target_compile_definitions(stfs-${_target} PRIVATE USE_CORO_DONE_TEST=1)
endforeach()

include_directories(../../include)

foreach(_target
    p1500_corolib
    p1500e_corolib
    p1510_corolib
    p1510e_corolib
    p1520_corolib
    p1520e_corolib
  )
  add_executable(stfs-${_target} ${_target}.cpp)
  target_link_libraries(stfs-${_target} corolib)
endforeach()

foreach(_target
    p1000_sa
    p1000e_sa
    p1010_sa
    p1010e_sa
    p1020_sa
    p1020e_sa

    p1100_sn
    p1100e_sn
    p1110_sn
    p1110e_sn
    p1120_sn
    p1120e_sn

    p1150_sn2
    p1150e_sn2
    p1160_sn2
    p1160e_sn2
    p1170_sn2
    p1170e_sn2

    p1200_void
    p1200e_void
    p1210_void
    p1210e_void
    p1220_void
    p1220e_void

    p1300_bool
    p1300e_bool
    p1310_bool
    p1310e_bool
    p1320_bool
    p1320e_bool

    p1400_coroutine_handle
    p1400e_coroutine_handle
    p1410_coroutine_handle
    p1410e_coroutine_handle
    p1420_coroutine_handle
    p1420e_coroutine_handle
  )
  add_executable(stfs2-${_target} ${_target}.cpp tracker.cpp print.cpp)
  target_compile_definitions(stfs2-${_target} PRIVATE USE_CORO_DONE_TEST=0)
endforeach()
