project (tutorialtests)

include_directories(../include ../examples/tutorial)

add_executable(p140X_tests p140X_tests.cpp ../examples/tutorial/p1400.cpp ../examples/tutorial/eventqueue.cpp ../examples/tutorial/eventqueuethr.cpp)
target_link_libraries(p140X_tests corolib gtest_main)

add_test(NAME tutorialtests COMMAND ${PROJECT_NAME})
