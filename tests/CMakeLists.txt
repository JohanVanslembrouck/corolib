project (tutorialtests)

include_directories(../include ../examples/tutorial)

set(target_sources
    p1400.cpp
    p1410.cpp
    p1420.cpp
    p1430.cpp
    p1430a.cpp
    p1440.cpp
    p1450.cpp
    p1460.cpp
    class01.cpp
    class02.cpp
    eventqueue.cpp
    eventqueuethr.cpp
)
list(TRANSFORM target_sources PREPEND "../examples/tutorial/")

add_executable(p140X_tests p140X_tests.cpp ${target_sources})
target_link_libraries(p140X_tests corolib gtest_main)

add_test(NAME tutorialtests COMMAND ${PROJECT_NAME})
