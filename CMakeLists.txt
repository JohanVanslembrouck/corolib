cmake_minimum_required(VERSION 3.17)

set(Boost_INCLUDE_DIR C:/Boost/include/boost-1_70)
set(Boost_LIBRARY_DIR C:/Boost/lib)

project (corolib)

add_subdirectory(examples/clientserver1)
add_subdirectory(examples/clientserver2)
add_subdirectory(examples/clientserver3)
