project (qt5examples)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Please adjust the following two variables to your Qt installation
set(USE_QT5 FALSE)
set(USE_QT6 TRUE)

if(${USE_QT5})
    find_package(Qt5 REQUIRED COMPONENTS Widgets Gui Core Network)

    include_directories(${Qt5_INCLUDE_DIRS}
                        ${Qt5Core_INCLUDE_DIRS}
                        ${Qt5Gui_INCLUDE_DIRS}
                        ${Qt5Widgets_INCLUDE_DIRS}
                        ${Qt5Network_INCLUDE_DIRS})
    link_directories(${Qt5_LIBRARY_DIR})
endif()

if(${USE_QT6})
    if(MSVC)
        add_compile_options(/Zc:__cplusplus)
    endif()

    find_package(Qt6 REQUIRED COMPONENTS Widgets Gui Core Network)

    include_directories(${Qt6_INCLUDE_DIRS}
                        ${Qt6Core_INCLUDE_DIRS}
                        ${Qt6Gui_INCLUDE_DIRS}
                        ${Qt6Widgets_INCLUDE_DIRS}
                        ${Qt6Network_INCLUDE_DIRS})

    link_directories(${Qt6_LIBRARY_DIR})
endif()

include(CMakePackageConfigHelpers)

set(CMAKE_AUTOMOC ON)

if(${USE_QT5})
  set(CL_QT_VERSION "Qt5")
endif()
if(${USE_QT6})
  set(CL_QT_VERSION "Qt6")
endif()

add_subdirectory(common)
add_subdirectory(clientserver10)
add_subdirectory(clientserver11)
add_subdirectory(clientserver12)
add_subdirectory(various)
