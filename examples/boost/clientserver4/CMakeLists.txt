project (clientserver4)

include_directories(../../../include ../common ${Boost_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIR})

foreach(_target
    client1
    server server2
  )
  add_executable(cs4-${_target} ${_target}.cpp)
  target_link_libraries(cs4-${_target} corolib corolib-boost)
endforeach()
