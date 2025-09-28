
project (example1)

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
# For boost: /D _WIN32_WINNT=0x0601
    set(CMAKE_CXX_FLAGS "/await:strict /EHsc")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_CXX_FLAGS "-Wall -fcoroutines")
else()
    message("Unsupported compiler")
endif()

include_directories(../../include)

add_executable(p1000-cfsms p1000-cfsms.cpp)

add_executable(p1010-cfsms-co p1010-cfsms-co.cpp)
target_link_libraries(p1010-cfsms-co corolib)

add_executable(p1012-cfsms-co p1012-cfsms-co.cpp)
target_link_libraries(p1012-cfsms-co corolib)

add_executable(p1014-cfsms-co p1014-cfsms-co.cpp)
target_link_libraries(p1014-cfsms-co corolib)
