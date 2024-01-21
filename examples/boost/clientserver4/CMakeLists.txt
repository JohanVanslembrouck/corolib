project (clientserver4)

include_directories(../../../include ../common)

foreach(_target
    client1
    server server2
  )
  add_executable(cs4-${_target} ${_target}.cpp)
  target_link_libraries(cs4-${_target} corolib corolib-boost)
endforeach()
