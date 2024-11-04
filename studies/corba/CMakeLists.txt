
project(corba_example)

cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

include_directories(../../include)

add_executable(corba_client_app corba_client_app.cpp interfacea_coroutine.cpp interfacea.cpp eventqueue.cpp)
target_link_libraries(corba_client_app corolib)
