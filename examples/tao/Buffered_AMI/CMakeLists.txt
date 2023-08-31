cmake_minimum_required(VERSION 3.14)

project (buffered_ami)

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CMAKE_CXX_FLAGS "/await:strict /EHsc /D WIN32=1")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_CXX_FLAGS "-Wall -fcoroutines")
else()
    message("Unsupported compiler")
endif()

if (WIN32)
    set(TAO_INCLUDE_DIRS C:/local/ACE_wrappers C:/local/ACE_wrappers/TAO)
    set(TAO_LIBRARY_DIR C:/local/ACE_wrappers/lib)
elseif (UNIX)
    set(TAO_INCLUDE_DIRS $ENV{HOME}/ACE_wrappers $ENV{HOME}/ACE_wrappers/TAO)
    set(TAO_LIBRARY_DIR $ENV{HOME}/ACE_wrappers/lib)
else()
    message("Unsupported operating system")
endif()

SET(COROLIB_ROOT_DIR ../../..)
include_directories(${COROLIB_ROOT_DIR}/include)

# IDL file
get_filename_component(idlfile "./test.idl" ABSOLUTE)

add_custom_command(
      OUTPUT testC.h testS.h testC.inl testC.cpp testS.cpp
      COMMAND tao_idl ARGS -GC "${idlfile}"
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/testC.h ${CMAKE_CURRENT_SOURCE_DIR}/testC.h
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/testS.h ${CMAKE_CURRENT_SOURCE_DIR}/testS.h
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/testC.inl ${CMAKE_CURRENT_SOURCE_DIR}/testC.inl
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/testC.cpp ${CMAKE_CURRENT_SOURCE_DIR}/testC.cpp
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/testS.cpp ${CMAKE_CURRENT_SOURCE_DIR}/testS.cpp
      DEPENDS "${idlfile}")

if (WIN32)
    set(TAO_LIBRARIES ${TAO_LIBRARY_DIR}/ACEd.lib
        ${TAO_LIBRARY_DIR}/TAOd.lib
        ${TAO_LIBRARY_DIR}/TAO_AnyTypeCoded.lib
        ${TAO_LIBRARY_DIR}/TAO_Valuetyped.lib
        ${TAO_LIBRARY_DIR}/TAO_PortableServerd.lib
        ${TAO_LIBRARY_DIR}/TAO_CodecFactoryd.lib
        ${TAO_LIBRARY_DIR}/TAO_PId.lib
        ${TAO_LIBRARY_DIR}/TAO_Messagingd.lib)
elseif (UNIX)
     set(TAO_LIBRARIES ${TAO_LIBRARY_DIR}/libACE.so
        ${TAO_LIBRARY_DIR}/libTAO.so
        ${TAO_LIBRARY_DIR}/libTAO_AnyTypeCode.so
        ${TAO_LIBRARY_DIR}/libTAO_Valuetype.so
        ${TAO_LIBRARY_DIR}/libTAO_PortableServer.so
        ${TAO_LIBRARY_DIR}/libTAO_CodecFactory.so
        ${TAO_LIBRARY_DIR}/libTAO_PI.so
        ${TAO_LIBRARY_DIR}/libTAO_Messaging.so)

else()
    message("Unsupported operating system")
endif()

include_directories(${TAO_INCLUDE_DIRS})

link_directories(${TAO_LIBRARY_DIR})

add_executable(ami_client client.cpp testC.cpp testS.cpp)
target_link_libraries(ami_client ${TAO_LIBRARIES})

add_executable(ami_client_coroutine client_coroutine.cpp testC.cpp testS.cpp)
target_link_libraries(ami_client_coroutine ${TAO_LIBRARIES} corolib)

add_executable(ami_client_coroutine2 client_coroutine2.cpp testC.cpp testS.cpp)
target_link_libraries(ami_client_coroutine2 ${TAO_LIBRARIES} corolib)

add_executable(ami_server server.cpp testC.cpp testS.cpp test_i.cpp)
target_link_libraries(ami_server ${TAO_LIBRARIES})
