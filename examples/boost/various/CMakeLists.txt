project (timers)

include_directories(../../../include ../common)

add_executable(timer01 timermain01.cpp timer01.cpp)
target_link_libraries(timer01 corolib)

add_executable(timer02 timermain02.cpp timer02.cpp)
target_link_libraries(timer02 corolib)

add_executable(timer03 timermain03.cpp timer03.cpp)
target_link_libraries(timer03 corolib)

add_executable(timer04 timermain04.cpp timer04.cpp)
target_link_libraries(timer04 corolib)

add_executable(timer05_async_queue timermain05_async_queue.cpp timer05_async_queue.cpp)
target_link_libraries(timer05_async_queue corolib)

add_executable(timer06_async_semaphore timermain06_async_semaphore.cpp timer06_async_semaphore.cpp)
target_link_libraries(timer06_async_semaphore corolib)

add_executable(timer07 timermain07.cpp timer07.cpp)
target_link_libraries(timer07 corolib)

add_executable(timer08 timermain08.cpp timer08.cpp)
target_link_libraries(timer08 corolib)
