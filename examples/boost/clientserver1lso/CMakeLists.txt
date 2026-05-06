project (clientserver1lso)

include_directories(../../../include ../commonlso)

foreach(_target
    client1 client1a
    clientserver
    server
  )
  add_executable(cs1elso-${_target} ${_target}.cpp)
  target_compile_definitions(cs1elso-${_target} PRIVATE USE_LAZY_START_TASKS=0)
  target_link_libraries(cs1elso-${_target} corolib corolib-boostlso)
endforeach()

foreach(_target
    client1 client1a
    clientserver
    server
  )
  add_executable(cs1llso-${_target} ${_target}.cpp)
  target_compile_definitions(cs1llso-${_target} PRIVATE USE_LAZY_START_TASKS=1)
  target_link_libraries(cs1llso-${_target} corolib corolib-boostlso)
endforeach()
