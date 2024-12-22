project (boostexamples)

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
else()
   message("Boost not found")
endif()

link_directories(${Boost_LIBRARY_DIRS})

add_subdirectory(common)
add_subdirectory(clientserver0)
add_subdirectory(clientserver1)
add_subdirectory(clientserver2)
add_subdirectory(clientserver3)
add_subdirectory(clientserver4)
add_subdirectory(various)
