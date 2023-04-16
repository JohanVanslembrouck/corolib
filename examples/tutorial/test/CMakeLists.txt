project (tutorialtest)

include_directories(../../../include)

add_executable(p1400_tests "p1400_tests.cpp" ../p1400.cpp ../eventqueue.cpp ../eventqueuethr.cpp)
target_link_libraries(p1400_tests corolib gtest_main)
