project (cppocoroexamplescl)

include_directories(../include ../../../include)

add_executable(cl-cd_client2 cd_client2.cpp addressfile.cpp)
target_link_libraries(cl-cd_client2 cppcorolib corolib synchronization wsock32 ws2_32)

add_executable(cl-cd_server2 cd_server2.cpp addressfile.cpp)
target_link_libraries(cl-cd_server2 cppcorolib corolib synchronization wsock32 ws2_32)


add_executable(cl-echo_client2 echo_client2.cpp addressfile.cpp)
target_link_libraries(cl-echo_client2 cppcorolib corolib synchronization wsock32 ws2_32)

add_executable(cl-echo_server2 echo_server2.cpp addressfile.cpp)
target_link_libraries(cl-echo_server2 cppcorolib corolib synchronization wsock32 ws2_32)

