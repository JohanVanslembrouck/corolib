# Copyright 2018 gRPC authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# cmake build file for C++ helloworld example.
# Assumes protobuf and gRPC have been installed using cmake.
# See cmake_externalproject/CMakeLists.txt for all-in-one cmake build
# that automatically builds all the dependencies before building helloworld.

cmake_minimum_required(VERSION 3.5.1)

project(HelloWorld C CXX)

include(../cmake/common.cmake)

include_directories(../../../../include)

# Proto file
get_filename_component(hw_proto "../../protos/helloworld.proto" ABSOLUTE)
get_filename_component(hw_proto_path "${hw_proto}" PATH)

# Generated sources
set(hw_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/helloworld.pb.cc")
set(hw_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/helloworld.pb.h")
set(hw_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/helloworld.grpc.pb.cc")
set(hw_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/helloworld.grpc.pb.h")
add_custom_command(
      OUTPUT "${hw_proto_srcs}" "${hw_proto_hdrs}" "${hw_grpc_srcs}" "${hw_grpc_hdrs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${hw_proto_path}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${hw_proto}"
      DEPENDS "${hw_proto}")

# Include generated *.pb.h files
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

# hw_grpc_proto
add_library(hw_grpc_proto
  ${hw_grpc_srcs}
  ${hw_grpc_hdrs}
  ${hw_proto_srcs}
  ${hw_proto_hdrs})
target_link_libraries(hw_grpc_proto
  corolib
  ${_REFLECTION}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF})

# Targets greeter_[async_](client|server)
foreach(_target
  greeter_client
  greeter_server
  greeter_async_client
  greeter_async_client2
  greeter_callback_client
  greeter_callback_server
  greeter_async_server)
  add_executable(${_target} "${_target}.cc")
  target_link_libraries(${_target}
    hw_grpc_proto
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF})
endforeach()

foreach(_target
  greeter_cb_coroutine_client
  greeter_cb_coroutine_client2
  greeter_cb_coroutine_client3)
  add_executable(${_target}_xe "${_target}.cc")
  target_compile_definitions(${_target}_xe PRIVATE USE_LAZY_START_OPS=0)
  target_link_libraries(${_target}_xe
    hw_grpc_proto
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF})
endforeach()

foreach(_target
  greeter_cb_coroutine_client
  greeter_cb_coroutine_client2
  greeter_cb_coroutine_client3)
  add_executable(${_target}_xl "${_target}.cc")
  target_compile_definitions(${_target}_xl PRIVATE USE_LAZY_START_OPS=1)
  target_link_libraries(${_target}_xl
    hw_grpc_proto
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF})
endforeach()

foreach(_target
  greeter_coroutine_client
  greeter_coroutine_client2
  greeter_coroutine_client2a)
  add_executable(${_target}_ee "${_target}.cc")
  target_compile_definitions(${_target}_ee PRIVATE USE_LAZY_START_TASKS=0 USE_LAZY_START_OPS=0)
  target_link_libraries(${_target}_ee
    hw_grpc_proto
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF})
endforeach()

foreach(_target
  greeter_coroutine_client
  greeter_coroutine_client2
  greeter_coroutine_client2a)
  add_executable(${_target}_el "${_target}.cc")
  target_compile_definitions(${_target}_el PRIVATE USE_LAZY_START_TASKS=0 USE_LAZY_START_OPS=1)
  target_link_libraries(${_target}_el
    hw_grpc_proto
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF})
endforeach()

foreach(_target
  greeter_coroutine_client
  greeter_coroutine_client2
  greeter_coroutine_client2a)
  add_executable(${_target}_le "${_target}.cc")
  target_compile_definitions(${_target}_le PRIVATE USE_LAZY_START_TASKS=1 USE_LAZY_START_OPS=0)
  target_link_libraries(${_target}_le
    hw_grpc_proto
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF})
endforeach()

foreach(_target
  greeter_coroutine_client
  greeter_coroutine_client2
  greeter_coroutine_client2a)
  add_executable(${_target}_ll "${_target}.cc")
  target_compile_definitions(${_target}_ll PRIVATE USE_LAZY_START_TASKS=1 USE_LAZY_START_OPS=1)
  target_link_libraries(${_target}_ll
    hw_grpc_proto
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF})
endforeach()
