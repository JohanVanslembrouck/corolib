project (clientserver3)

include_directories(../../../include ../common ${Boost_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIR})

foreach(_target
    client1 client2
    server server2 server3 server4 server5 server6 server7 server8 server8a server9 server9a
  )
  add_executable(cs3-${_target} ${_target}.cpp)
  target_link_libraries(cs3-${_target} corolib corolib-boost)
endforeach()
