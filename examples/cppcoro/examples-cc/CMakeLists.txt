project (cppocoroexamples)

include_directories(../include)

#[[
https://stackoverflow.com/questions/41430077/waitonaddress-unresolved
https://stackoverflow.com/questions/15119639/how-to-link-winsock-in-cmake
]]

add_executable(cc-cd_client1 cd_client1.cpp addressfile.cpp)
target_link_libraries(cc-cd_client1 cppcorolib synchronization wsock32 ws2_32)

add_executable(cc-cd_client2 cd_client2.cpp addressfile.cpp)
target_link_libraries(cc-cd_client2 cppcorolib synchronization wsock32 ws2_32)

add_executable(cc-cd_server1 cd_server1.cpp addressfile.cpp)
target_link_libraries(cc-cd_server1 cppcorolib synchronization wsock32 ws2_32)

add_executable(cc-cd_server2 cd_server2.cpp addressfile.cpp)
target_link_libraries(cc-cd_server2 cppcorolib synchronization wsock32 ws2_32)


add_executable(cc-echo_client1 echo_client1.cpp addressfile.cpp)
target_link_libraries(cc-echo_client1 cppcorolib synchronization wsock32 ws2_32)

add_executable(cc-echo_client2 echo_client2.cpp addressfile.cpp)
target_link_libraries(cc-echo_client2 cppcorolib synchronization wsock32 ws2_32)

add_executable(cc-echo_server1 echo_server1.cpp addressfile.cpp)
target_link_libraries(cc-echo_server1 cppcorolib synchronization wsock32 ws2_32)

add_executable(cc-echo_server2 echo_server2.cpp addressfile.cpp)
target_link_libraries(cc-echo_server2 cppcorolib synchronization wsock32 ws2_32)

