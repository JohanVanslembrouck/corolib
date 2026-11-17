project (tutorialtests)

include_directories(../include ../examples/tutorial)

add_executable(p11XX_tests p11XX_tests.cpp)
target_link_libraries(p11XX_tests corolib gtest_main)

set(target_sources
    p1400.cpp
    p1410.cpp
    p1420.cpp
    p1430.cpp
    p1430a.cpp
    p1440.cpp
    p1450.cpp
    p1460.cpp
	
	p1700.cpp
    p1730.cpp
    p1730a.cpp

    p1800.cpp
    p1810.cpp
    p1820.cpp
    p1830.cpp
    p1840.cpp

    class00.cpp
    class01.cpp
    class02.cpp
	
    eventqueue.cpp
    eventqueuethr.cpp
)
list(TRANSFORM target_sources PREPEND "../examples/tutorial/")

add_executable(p14XX_tests p14XX_tests.cpp ${target_sources})
target_link_libraries(p14XX_tests corolib gtest_main)

add_executable(p17XX_tests p17XX_tests.cpp ${target_sources})
target_link_libraries(p17XX_tests corolib gtest_main)

add_executable(p18XX_tests p18XX_tests.cpp ${target_sources})
target_link_libraries(p18XX_tests corolib gtest_main)

add_executable(p22XX_tests p22XX_tests.cpp ${target_sources})
target_link_libraries(p22XX_tests corolib gtest_main)

add_test(NAME tutorialtests COMMAND ${PROJECT_NAME})
