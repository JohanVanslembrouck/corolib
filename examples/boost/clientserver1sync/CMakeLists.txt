project (clientserver1sync)

include_directories(../../../include ../commonsync)

foreach(_target
    client0
    client1
    client3WA
  )
  add_executable(cs1s-${_target} ${_target}.cpp)
  target_link_libraries(cs1s-${_target} corolib corolib-boostsync)
endforeach()
