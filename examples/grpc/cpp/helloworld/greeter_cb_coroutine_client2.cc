/**
 * @file greeter_cb_coroutine_client2.cc
 * @brief Added coroutine implementation.
 * Based on the implementation in greeter_callback_client.cc and greeter_cb_coroutine_client.cc.
 * In this variant start_SayHello returns async_operation<Status> instead of async_operation<void>.
 * Consequently, there is no need to pass Status as reference argument to start_SayHello.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

/*
 *
 * Copyright 2021 gRPC authors.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include <grpcpp/grpcpp.h>

#include <corolib/print.h>
#include <corolib/commservice.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

#ifdef BAZEL_BUILD
#include "examples/protos/helloworld.grpc.pb.h"
#else
#include "helloworld.grpc.pb.h"
#endif

#define USE_ORIGINAL 0

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using helloworld::Greeter;
using helloworld::HelloReply;
using helloworld::HelloRequest;

using namespace corolib;

class GreeterClient : public CommService {
 public:
  GreeterClient(std::shared_ptr<Channel> channel)
      : stub_(Greeter::NewStub(channel)) {}

#if USE_ORIGINAL
  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  std::string SayHello(const std::string& user) {
    // Data we are sending to the server.
    HelloRequest request;
    request.set_name(user);

    // Container for the data we expect from the server.
    HelloReply reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // The actual RPC.
    std::mutex mu;
    std::condition_variable cv;
    bool done = false;
    Status status;
    stub_->async()->SayHello(&context, &request, &reply,
                             [&mu, &cv, &done, &status](Status s) {
                               status = std::move(s);
                               std::lock_guard<std::mutex> lock(mu);
                               done = true;
                               cv.notify_one();
                             });

    std::unique_lock<std::mutex> lock(mu);
    while (!done) {
      cv.wait(lock);
    }

    // Act upon its status.
    if (status.ok()) {
      return reply.message();
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
      return "RPC failed";
    }
  }
#else
  async_task<std::string> SayHelloAsync(const std::string& user) {
      // Data we are sending to the server.
      HelloRequest request;
      request.set_name(user);

      // Container for the data we expect from the server.
      HelloReply reply;

      // Context for the client. It could be used to convey extra information to
      // the server and/or tweak certain RPC behaviors.
      ClientContext context;

      // Storage for the status of the RPC upon completion.
      Status status;
      status = co_await start_SayHello(&context, request, reply);

      // Act upon the status of the actual RPC.
      if (status.ok()) {
          co_return reply.message();
      }
      else {
          co_return "RPC failed";
      }
  }

  async_operation<Status> start_SayHello(ClientContext* pcontext, HelloRequest& request, HelloReply& reply) {
      int index = get_free_index();
      async_operation<Status> ret{ this, index };
      start_SayHello_impl(index, pcontext, request, reply);
      return ret;
  }

  void start_SayHello_impl(int idx, ClientContext* pcontext, HelloRequest& request, HelloReply& reply) {
      stub_->async()->SayHello(pcontext, &request, &reply,
          [idx, this](Status s) {
              Status status = std::move(s);
             
              async_operation_base* om_async_operation = get_async_operation(idx);
              async_operation<Status>* om_async_operation_t =
                  dynamic_cast<async_operation<Status>*>(om_async_operation);
              if (om_async_operation_t) {
                  om_async_operation_t->set_result(status);
                  om_async_operation_t->completed();
              }

          });
  }
#endif

 private:
  std::unique_ptr<Greeter::Stub> stub_;
};

int main(int argc, char** argv) {
  // Instantiate the client. It requires a channel, out of which the actual RPCs
  // are created. This channel models a connection to an endpoint specified by
  // the argument "--target=" which is the only expected argument.
  // We indicate that the channel isn't authenticated (use of
  // InsecureChannelCredentials()).
  std::string target_str;
  std::string arg_str("--target");
  if (argc > 1) {
    std::string arg_val = argv[1];
    size_t start_pos = arg_val.find(arg_str);
    if (start_pos != std::string::npos) {
      start_pos += arg_str.size();
      if (arg_val[start_pos] == '=') {
        target_str = arg_val.substr(start_pos + 1);
      } else {
        std::cout << "The only correct argument syntax is --target="
                  << std::endl;
        return 0;
      }
    } else {
      std::cout << "The only acceptable argument is --target=" << std::endl;
      return 0;
    }
  } else {
    target_str = "localhost:50051";
  }
  GreeterClient greeter(
      grpc::CreateChannel(target_str, grpc::InsecureChannelCredentials()));

#if USE_ORIGINAL
  std::string user("world");
  std::string reply = greeter.SayHello(user);
  std::cout << "Greeter received: " << reply << std::endl;
#else
  std::string user("coroutine world");
  async_task<std::string> t = greeter.SayHelloAsync(user);
  std::string str = t.get_result();
  std::cout << "Greeter received: " << str << std::endl;
#endif

  return 0;
}
