project (clientserver1)

include_directories(../../../include ../common)

foreach(_target
    client1 client1a client1b client1c
    client3 client3WA client3WAny
    client4obs client4obs2 client4obs3
    client5thr
    clientserver
    server
  )
  add_executable(cs1-${_target} ${_target}.cpp)
  target_compile_definitions(cs1-${_target} PRIVATE USE_LAZY_START_TASKS=0)
  target_link_libraries(cs1-${_target} corolib corolib-boost)
endforeach()

foreach(_target
    client1 client1a
    client3WA
    client3WAny
    clientserver
    server
  )
  add_executable(cs1l-${_target} ${_target}.cpp)
  target_compile_definitions(cs1l-${_target} PRIVATE USE_LAZY_START_TASKS=1)
  target_link_libraries(cs1l-${_target} corolib corolib-boost)
endforeach()

