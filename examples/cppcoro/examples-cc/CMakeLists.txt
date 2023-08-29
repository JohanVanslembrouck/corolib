project (cppocoroexamples)

include_directories(../include .)

#[[
https://stackoverflow.com/questions/41430077/waitonaddress-unresolved
https://stackoverflow.com/questions/15119639/how-to-link-winsock-in-cmake
]]

foreach(_target
    cd_client1 cd_client2
    cd_server1 cd_server2
    echo_client1 echo_client2
    echo_server1 echo_server2
    udp_client1 udp_client2
    udp_server1 udp_server2
  )
  add_executable(cc-${_target} ${_target}.cpp addressfile.cpp)
  target_link_libraries(cc-${_target} cppcorolib synchronization wsock32 ws2_32)
endforeach()

add_executable(cc-file_write_read1 file_write_read1)
target_link_libraries(cc-file_write_read1 cppcorolib synchronization ws2_32)

add_executable(cc-file_write_read2 file_write_read2)
target_link_libraries(cc-file_write_read2 cppcorolib synchronization ws2_32)