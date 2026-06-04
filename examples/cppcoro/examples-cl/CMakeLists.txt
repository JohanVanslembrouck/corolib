project (cppocoroexamplescl)

include_directories(../include ../../../include)

foreach(_target
    cd_server2_client2
    echo_server2_client2
    udp_server2_client2
  )
if(WIN32)
  add_executable(cl-${_target} ${_target}.cpp)
  target_link_libraries(cl-${_target} cppcorolib corolib synchronization wsock32 ws2_32)
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  add_executable(cl-${_target} ${_target}.cpp)
  target_link_libraries(cl-${_target} cppcorolib corolib)
endif()
endforeach()

foreach(_target
    cd_server2 cd_client2
    echo_server2 echo_client2
    udp_server2 udp_client2
  )
if(WIN32)
  add_executable(cl-${_target} ${_target}.cpp addressfile.cpp)
  target_link_libraries(cl-${_target} cppcorolib corolib synchronization wsock32 ws2_32)
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  add_executable(cl-${_target} ${_target}.cpp addressfile.cpp)
  target_link_libraries(cl-${_target} cppcorolib corolib)
endif()
endforeach()

if(WIN32)
add_executable(cl-file_write_read2 file_write_read2.cpp)
target_link_libraries(cl-file_write_read2 cppcorolib corolib synchronization ws2_32)
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
add_executable(cl-file_write_read2 file_write_read2.cpp)
target_link_libraries(cl-file_write_read2 cppcorolib corolib)
endif()
