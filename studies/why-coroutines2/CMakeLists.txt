project (why-coroutines2)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# USE_IN_MT_APPS can be set to 0 to build corolib for single-threaded frameworks and applications.

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CMAKE_CXX_FLAGS "/await:strict /EHsc /D USE_CPP20SEMAPHORE=1 /D USE_IN_MT_APPS=1")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    # -lstdc++ had to be added to build with gcc 13.3.0 on Ubuntu 24.04.
    # This link option is not necessary to build with gcc 11.4.0 on Ubuntu 22.04.
    set(CMAKE_CXX_FLAGS "-Wall -fPIC -fcoroutines -D USE_CPP20SEMAPHORE=1 -D USE_IN_MT_APPS=1 -lstdc++")
else()
    message("Unsupported compiler")
endif()

include_directories(../../include)

add_executable(p1000-sync p1000-sync.cpp ../../lib/print.cpp)
add_executable(p1001-sync p1001-sync.cpp ../../lib/print.cpp)

add_executable(p1010-sync-thread p1010-sync-thread.cpp ../../lib/print.cpp)
add_executable(p1011-sync-thread p1011-sync-thread.cpp ../../lib/print.cpp)

add_executable(p1020-async p1020-async.cpp eventqueue.cpp ../../lib/print.cpp)
add_executable(p1021-async p1021-async.cpp eventqueue.cpp ../../lib/print.cpp)
add_executable(p1025-async p1025-async.cpp eventqueue.cpp ../../lib/print.cpp)
add_executable(p1026-async p1026-async.cpp eventqueue.cpp ../../lib/print.cpp)

add_executable(p1030-async-thread p1030-async-thread.cpp eventqueuethr.cpp ../../lib/print.cpp)
add_executable(p1031-async-thread p1031-async-thread.cpp eventqueuethr.cpp ../../lib/print.cpp) 

add_executable(p1040-coroutine p1040-coroutine.cpp eventqueue.cpp ../../lib/print.cpp)
add_executable(p1041-coroutine p1041-coroutine.cpp eventqueue.cpp ../../lib/print.cpp)
add_executable(p1042-coroutine p1042-coroutine.cpp eventqueue.cpp ../../lib/print.cpp)

add_executable(p1060-corolib p1060-corolib.cpp eventqueue.cpp)
target_link_libraries(p1060-corolib corolib)
add_executable(p1061-corolib p1061-corolib.cpp eventqueue.cpp)
target_link_libraries(p1061-corolib corolib)
add_executable(p1062-corolib p1062-corolib.cpp eventqueue.cpp)
target_link_libraries(p1062-corolib corolib)

add_executable(p1070-corolib-thread p1070-corolib-thread.cpp eventqueuethr.cpp)
target_link_libraries(p1070-corolib-thread corolib)
add_executable(p1071-corolib-thread p1071-corolib-thread.cpp eventqueuethr.cpp)
target_link_libraries(p1071-corolib-thread corolib)
add_executable(p1072-corolib-thread p1072-corolib-thread.cpp eventqueuethr.cpp)
target_link_libraries(p1072-corolib-thread corolib)
