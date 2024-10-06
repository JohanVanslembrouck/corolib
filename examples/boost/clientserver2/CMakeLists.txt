project (clientserver2)

include_directories(../../../include ../common)

foreach(_target
    client1 client1a 
    server server2
    server-th server2-th
  )
  add_executable(cs2-${_target} ${_target}.cpp)
  target_link_libraries(cs2-${_target} corolib corolib-boost)
endforeach()

