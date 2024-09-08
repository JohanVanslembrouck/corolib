project (curl-examples)

include_directories(../../../include)

if (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")

include_directories(C:/local/curl-8.9.1/include)
link_directories(C:/local/curl-8.9.1/build/lib/Release)

set(CURLLIB libcurl_imp)

elseif (CMAKE_CXX_COMPILER_ID MATCHES "GNU")

set(CURLLIB curl)

else()
    message("Unsupported compiler")
endif()


add_executable(multi-single multi-single.cpp)
target_link_libraries(multi-single ${CURLLIB})
add_executable(multi-single-class multi-single-class.cpp)
target_link_libraries(multi-single-class ${CURLLIB})
add_executable(multi-single-coroutine multi-single-coroutine.cpp)
target_link_libraries(multi-single-coroutine ${CURLLIB} corolib)

add_executable(multi-double multi-double.cpp)
target_link_libraries(multi-double ${CURLLIB})
add_executable(multi-double-class multi-double-class.cpp)
target_link_libraries(multi-double-class ${CURLLIB})
add_executable(multi-double-coroutine multi-double-coroutine.cpp)
target_link_libraries(multi-double-coroutine ${CURLLIB} corolib)
