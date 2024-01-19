project (qt5examples)

find_package(Qt5 REQUIRED COMPONENTS Widgets Gui Core Network)

include_directories(${Qt5_INCLUDE_DIRS}
                    ${Qt5Core_INCLUDE_DIRS}
                    ${Qt5Gui_INCLUDE_DIRS}
                    ${Qt5Widgets_INCLUDE_DIRS}
                    ${Qt5Network_INCLUDE_DIRS})

link_directories(${Qt5_LIBRARY_DIR})

include(CMakePackageConfigHelpers)

set(CMAKE_AUTOMOC ON)

add_subdirectory(common)
add_subdirectory(clientserver11)
add_subdirectory(clientserver12)
add_subdirectory(various)
