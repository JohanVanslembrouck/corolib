project (clientserver0)

foreach(_target
    client1 client2
    server
  )
  add_executable(cs0-${_target} ${_target}.cpp print.cpp)
endforeach()
