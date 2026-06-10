project (cppocoroexamples)

include_directories(../include .)

#[[
https://stackoverflow.com/questions/41430077/waitonaddress-unresolved
https://stackoverflow.com/questions/15119639/how-to-link-winsock-in-cmake
]]

foreach(_target
    cd_server1_client1
    cd_server2_client2
    echo_server1_client1
    echo_server2_client2
    multiple_echo_server1_client1
    multiple_echo_server2_client2
    udp_server1_client1
    udp_server2_client2
  )
if(WIN32)
  add_executable(cc-${_target} ${_target}.cpp)
  target_link_libraries(cc-${_target} cppcorolib synchronization wsock32 ws2_32)
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  add_executable(cc-${_target} ${_target}.cpp)
  target_link_libraries(cc-${_target} cppcorolib)
endif()
endforeach()

foreach(_target
    cd_server1 cd_client1
    cd_server2 cd_client2
    echo_server1 echo_client1
    echo_server2 echo_client2
    multiple_echo_server1 multiple_echo_client1
    multiple_echo_server2 multiple_echo_client2
    udp_server1 udp_client1
    udp_server2 udp_client2
  )
if(WIN32)
  add_executable(cc-${_target} ${_target}.cpp addressfile.cpp)
  target_link_libraries(cc-${_target} cppcorolib synchronization wsock32 ws2_32)
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  add_executable(cc-${_target} ${_target}.cpp addressfile.cpp)
  target_link_libraries(cc-${_target} cppcorolib)
endif()
endforeach()

if(WIN32)
add_executable(cc-file_write_read1 file_write_read1.cpp)
target_link_libraries(cc-file_write_read1 cppcorolib synchronization ws2_32)
add_executable(cc-file_write_read2 file_write_read2.cpp)
target_link_libraries(cc-file_write_read2 cppcorolib synchronization ws2_32)
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
add_executable(cc-file_write_read1 file_write_read1.cpp)
target_link_libraries(cc-file_write_read1 cppcorolib)
add_executable(cc-file_write_read2 file_write_read2.cpp)
target_link_libraries(cc-file_write_read2 cppcorolib)
endif()
