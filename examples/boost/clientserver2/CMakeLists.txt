project (clientserver2)

include_directories(../../../include ../common ${Boost_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIR})

foreach(_target
    client1 client1a 
    server server2
  )
  add_executable(cs2-${_target} ${_target}.cpp)
  target_link_libraries(cs2-${_target} corolib corolib-boost)
endforeach()

