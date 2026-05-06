project (clientserver1x)

include_directories(../../../include ../common ../commonlso)

foreach(_target
    client1
    client1a
    clientserver
    server
  )
  add_executable(cs1xee-${_target} ${_target}.cpp)
  target_compile_definitions(cs1xee-${_target} PRIVATE USE_LAZY_START_TASKS=0 USE_LAZY_START_OPS=0)
  target_link_libraries(cs1xee-${_target} corolib corolib-boost)
endforeach()

foreach(_target
    client1
    client1a
    clientserver
    server
  )
  add_executable(cs1xel-${_target} ${_target}.cpp)
  target_compile_definitions(cs1xel-${_target} PRIVATE USE_LAZY_START_TASKS=0 USE_LAZY_START_OPS=1)
  target_link_libraries(cs1xel-${_target} corolib corolib-boostlso)
endforeach()

foreach(_target
    client1
    client1a
    clientserver
    server
  )
  add_executable(cs1xle-${_target} ${_target}.cpp)
  target_compile_definitions(cs1xle-${_target} PRIVATE USE_LAZY_START_TASKS=1 USE_LAZY_START_OPS=0)
  target_link_libraries(cs1xle-${_target} corolib corolib-boost)
endforeach()

foreach(_target
    client1
    client1a
    clientserver
    server
  )
  add_executable(cs1xll-${_target} ${_target}.cpp)
  target_compile_definitions(cs1xll-${_target} PRIVATE USE_LAZY_START_TASKS=1 USE_LAZY_START_OPS=1)
  target_link_libraries(cs1xll-${_target} corolib corolib-boostlso)
endforeach()
