cmake_minimum_required(VERSION 3.17)

set(Boost_INCLUDE_DIR C:/local/boost/boost_1_79_0)
set(Boost_LIBRARY_DIR C:/local/boost/boost_1_79_0/stage/lib)

project (corolib)

add_subdirectory(lib)

add_subdirectory(examples/clientserver1)
add_subdirectory(examples/clientserver2)
add_subdirectory(examples/clientserver3)
add_subdirectory(examples/clientserver4)

add_subdirectory(examples/various-boost)
add_subdirectory(examples/tutorial)
add_subdirectory(examples/why-coroutines)
add_subdirectory(examples/corolab)

