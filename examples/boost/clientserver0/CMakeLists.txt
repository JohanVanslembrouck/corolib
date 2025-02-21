project (clientserver0)

foreach(_target
    client0 client1 client2 client3
    server server1
  )
  add_executable(cs0-${_target} ${_target}.cpp print.cpp)
endforeach()
