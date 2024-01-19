project (qt5timers)

include_directories(../../../include
                    ../common)

add_executable(qt-timer01 timermain01.cpp timer01.cpp)
target_link_libraries(qt-timer01 corolib-qt5 corolib Qt5::Core Qt5::Network)

add_executable(qt-timer02 timermain02.cpp timer02.cpp)
target_link_libraries(qt-timer02 corolib-qt5 corolib Qt5::Core Qt5::Network)

add_executable(qt-timer03 timermain03.cpp timer03.cpp)
target_link_libraries(qt-timer03 corolib-qt5 corolib Qt5::Core Qt5::Network)
