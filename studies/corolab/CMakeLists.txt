project (corolab)

include_directories(../../include)

foreach(_target
    p0010 p0020 p0030 p0040 p0050 p0060
    p0100 p0110 p0120
    p0200
    p0300 p0310 p0320 
#   p0330 p0340 p0345
    p0400 p0403a p0403at p0403b p0403bt p0403c p0403ct p0403d p0403dt p0406
    p0414 p0415 p0415a p0416 p0417 p0418 p0419 p0420 p0422 p0423 p0424 p0425 p0426 p0427
    p0430 p0432 p0435 p0436
#   p0440 p450
    p0460 p0470 p0472
    p0500 p0510 p0520
  )
  add_executable(${_target} ${_target}.cpp tracker.cpp print.cpp)
endforeach()

if (BOOST_INSTALLED)

    if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    # For boost: /D _WIN32_WINNT=0x0601
        set(CMAKE_CXX_FLAGS "/await:strict /EHsc /D _WIN32_WINNT=0x0601")
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
        set(CMAKE_CXX_FLAGS "-Wall -fcoroutines")
    else()
        message("Unsupported compiler")
    endif()

    find_package(Boost)

    if(Boost_FOUND)
        include_directories(${Boost_INCLUDE_DIRS})
        link_directories(${Boost_LIBRARY_DIR})
    else()
       message("Boost not found")
    endif()

    foreach(_target
        p0700c p0700s p0710c
        p0800c p0800cs p0800s
        p0810c p0820c p0830c p0830s
      )
      add_executable(${_target} ${_target}.cpp tracker.cpp print.cpp)
    endforeach()

endif()
