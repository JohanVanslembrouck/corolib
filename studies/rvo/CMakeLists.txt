project(study-rvo)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(rvo1a rvo1.cpp)
add_executable(rvo2a rvo2.cpp)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(rvo1b rvo1.cpp)
add_executable(rvo2b rvo2.cpp)
