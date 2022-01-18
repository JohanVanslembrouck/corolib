cmake_minimum_required(VERSION 3.17)

project (tutorial)

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
	set(CMAKE_CXX_FLAGS "/await /EHsc /D _WIN32_WINNT=0x0601")
else()
	set(CMAKE_CXX_FLAGS "-Wall")
endif()

include_directories(../../include/corolib)

link_directories(${Boost_LIBRARY_DIR})

add_executable(p1000 p1000.cpp)
target_link_libraries(p1000 corolib)
add_executable(p1010-thread p1010-thread.cpp)
target_link_libraries(p1010-thread corolib)

add_executable(p1050-resume_same_thread p1050-resume_same_thread.cpp)
target_link_libraries(p1050-resume_same_thread corolib)
add_executable(p1060-resume_new_thread p1060-resume_new_thread.cpp)
target_link_libraries(p1060-resume_new_thread corolib)


add_executable(p1100-auto_reset_event p1100-auto_reset_event.cpp)
target_link_libraries(p1100-auto_reset_event corolib)
add_executable(p1110-auto_reset_event-oneway_task p1110-auto_reset_event-oneway_task.cpp)
target_link_libraries(p1110-auto_reset_event-oneway_task corolib)
add_executable(p1120-auto_reset_event-thread p1120-auto_reset_event-thread.cpp)
target_link_libraries(p1120-auto_reset_event-thread corolib)


add_executable(p1200-mini0 p1200-mini0.cpp)
target_link_libraries(p1200-mini0 corolib)
add_executable(p1210-mini1-thread p1210-mini1-thread.cpp)
target_link_libraries(p1210-mini1-thread corolib)
add_executable(p1220-mini1-oneway_task-thread p1220-mini1-oneway_task-thread.cpp)
target_link_libraries(p1220-mini1-oneway_task-thread corolib)

add_executable(p1300-future p1300-future.cpp)
target_link_libraries(p1300-future corolib)

add_executable(p1400-async_operation p1400-async_operation.cpp)
target_link_libraries(p1400-async_operation corolib)
add_executable(p1402-async_operation-eventqueue p1402-async_operation-eventqueue.cpp)
target_link_libraries(p1402-async_operation-eventqueue corolib)
add_executable(p1404-async_operation-thread p1404-async_operation-thread.cpp)
target_link_libraries(p1404-async_operation-thread corolib)

add_executable(p1410-async_operation p1410-async_operation.cpp class01.cpp)
target_link_libraries(p1410-async_operation corolib)
add_executable(p1412-async_operation-eventqueue p1412-async_operation-eventqueue.cpp class01.cpp)
target_link_libraries(p1412-async_operation-eventqueue corolib)
add_executable(p1414-async_operation-thread p1414-async_operation-thread.cpp class01.cpp)
target_link_libraries(p1414-async_operation-thread corolib)

add_executable(p1420-async_operation p1420-async_operation.cpp class01.cpp)
target_link_libraries(p1420-async_operation corolib)
add_executable(p1422-async_operation-eventqueue p1422-async_operation-eventqueue.cpp class01.cpp)
target_link_libraries(p1422-async_operation-eventqueue corolib)
add_executable(p1424-async_operation-thread p1424-async_operation-thread.cpp class01.cpp)
target_link_libraries(p1424-async_operation-thread corolib)

add_executable(p1430-async_operation p1430-async_operation.cpp class01.cpp)
target_link_libraries(p1430-async_operation corolib)
add_executable(p1432-async_operation-eventqueue p1432-async_operation-eventqueue.cpp class01.cpp)
target_link_libraries(p1432-async_operation-eventqueue corolib)
add_executable(p1434-async_operation-thread p1434-async_operation-thread.cpp class01.cpp)
target_link_libraries(p1434-async_operation-thread corolib)

add_executable(p1440-async_operation p1440-async_operation.cpp class01.cpp)
target_link_libraries(p1440-async_operation corolib)
add_executable(p1442-async_operation-eventqueue p1442-async_operation-eventqueue.cpp class01.cpp)
target_link_libraries(p1442-async_operation-eventqueue corolib)
add_executable(p1444-async_operation-thread p1444-async_operation-thread.cpp class01.cpp)
target_link_libraries(p1444-async_operation-thread corolib)

add_executable(p1450-async_operation p1450-async_operation.cpp class01.cpp)
target_link_libraries(p1450-async_operation corolib)
add_executable(p1452-async_operation-eventqueue p1452-async_operation-eventqueue.cpp class01.cpp)
target_link_libraries(p1452-async_operation-eventqueue corolib)
add_executable(p1454-async_operation-thread p1454-async_operation-thread.cpp class01.cpp)
target_link_libraries(p1454-async_operation-thread corolib)

add_executable(p1462-async_operation-eventqueue p1462-async_operation-eventqueue.cpp class02.cpp)
target_link_libraries(p1462-async_operation-eventqueue corolib)
add_executable(p1464-async_operation-thread p1464-async_operation-thread.cpp class02.cpp)
target_link_libraries(p1464-async_operation-thread corolib)
