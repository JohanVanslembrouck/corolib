project (clientserver2)

include_directories(../../../include ../common ${Boost_INCLUDE_DIR})

link_directories(${Boost_LIBRARY_DIR})

foreach(_target
    client1 client1a 
    server server2
  )
  add_executable(cs2-${_target} ${_target}.cpp)
  target_link_libraries(cs2-${_target} corolib corolib-boost)
endforeach()

#[[
add_executable(cs2-client1 client1.cpp)
target_link_libraries(cs2-client1 corolib corolib-boost)
add_executable(cs2-client1a client1a.cpp)
target_link_libraries(cs2-client1a corolib corolib-boost)
add_executable(cs2-server server.cpp)
target_link_libraries(cs2-server corolib corolib-boost)
add_executable(cs2-server2 server2.cpp)
target_link_libraries(cs2-server2 corolib corolib-boost)
]]
