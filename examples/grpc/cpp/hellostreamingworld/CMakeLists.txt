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

project(HelloWorldStream C CXX)

include(../cmake/common.cmake)

include_directories(../../../../include)

# Proto file
get_filename_component(hsw_proto "../../protos/hellostreamingworld.proto" ABSOLUTE)
get_filename_component(hsw_proto_path "${hsw_proto}" PATH)

# Generated sources
set(hsw_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/hellostreamingworld.pb.cc")
set(hsw_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/hellostreamingworld.pb.h")
set(hsw_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/hellostreamingworld.grpc.pb.cc")
set(hsw_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/hellostreamingworld.grpc.pb.h")
add_custom_command(
      OUTPUT "${hsw_proto_srcs}" "${hsw_proto_hdrs}" "${hsw_grpc_srcs}" "${hsw_grpc_hdrs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${hsw_proto_path}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${hsw_proto}"
      DEPENDS "${hsw_proto}")

# Include generated *.pb.h files
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

# hw_grpc_proto
add_library(hsw_grpc_proto
  ${hsw_grpc_srcs}
  ${hsw_grpc_hdrs}
  ${hsw_proto_srcs}
  ${hsw_proto_hdrs})
target_link_libraries(hsw_grpc_proto
  corolib
  ${_REFLECTION}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF})

# Targets greeter_[async_](client|server)
foreach(_target
  multigreeter_client
  multigreeter_coroutine_client
  multigreeter_coroutine_client2
  multigreeter_coroutine_client3
  multigreeter_server)
  add_executable(${_target} "${_target}.cc")
  target_link_libraries(${_target}
    hsw_grpc_proto
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF})
endforeach()
