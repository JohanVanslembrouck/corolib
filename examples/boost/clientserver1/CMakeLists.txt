project (clientserver1)

include_directories(../../../include ../common ${Boost_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIR})

foreach(_target
    client1 client1a client1b client1c
    client3 client3WA client3WAny
    client4obs client4obs2 client4obs3
    client5thr
    clientserver
    server
  )
  add_executable(cs1-${_target} ${_target}.cpp)
  target_link_libraries(cs1-${_target} corolib corolib-boost)
endforeach()

