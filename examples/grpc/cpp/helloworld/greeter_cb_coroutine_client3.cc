/**
 * @file greeter_cb_coroutine_client3.cc
 * @brief Added coroutine implementation.
 * Based on the implementation in greeter_callback_client.cc and greeter_cb_coroutine_client.cc.
 * 
 * In this variant start_SayHello returns async_operation<Status> instead of async_operation<void>.
 * Consequently, there is no need to pass Status as reference argument to start_SayHello.
 * 
 * In contrast to greeter_cb_coroutine_client2.cc, this implementation uses an event queue.
 * The callback function (that runs on a dedicated thread) pushes completionHandler onto the event queue.
 * It will be popped from the event queue by the main thread.
 * This minimizes the (application) code that runs on the completion thread.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
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
#include <corolib/eventqueue.h>

#ifdef BAZEL_BUILD
#include "examples/protos/helloworld.grpc.pb.h"
#else
#include "helloworld.grpc.pb.h"
#endif

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using helloworld::Greeter;
using helloworld::HelloReply;
using helloworld::HelloRequest;

using namespace corolib;

EventQueueFunctionVoidVoid eventQueueThr;

const int NR_ITERATIONS = 10;

class GreeterClient : public CommService {
 public:
  GreeterClient(std::shared_ptr<Channel> channel)
      : stub_(Greeter::NewStub(channel)) {}

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
    print(PRI1, "SayHello: pre\n");
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
    print(PRI1, "SayHello: post\n");

    // Act upon its status.
    if (status.ok()) {
      return reply.message();
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
      return "RPC failed";
    }
  }

  async_task<std::string> SayHelloCo(const std::string& user) {
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

      print(PRI1, "SayHelloCo: pre\n");
      status = co_await start_SayHello(&context, request, reply);
      print(PRI1, "SayHelloCo: post\n");

      // Act upon the status of the actual RPC.
      if (status.ok()) {
          co_return reply.message();
      }
      else {
          co_return "RPC failed";
      }
  }

  async_ltask<std::string> SayHelloCoL(const std::string& user) {
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

      print(PRI1, "SayHelloCoL: pre\n");
      status = co_await start_SayHello(&context, request, reply);
      print(PRI1, "SayHelloCoL: post\n");

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
      stub_->async()->SayHello(pcontext, &request, &reply,
          [index, this](Status s) {
              print(PRI1, "start_SayHello: handler\n");
              Status status = std::move(s);
              // Difference with reeter_cb_coroutine_client3.cc
              eventQueueThr.push(
                  [this, index, status]() {
                      this->completionHandler<Status>(index, status);
                  });
          });
      return ret;
  }

 private:
  std::unique_ptr<Greeter::Stub> stub_;
};

void runSayHello(GreeterClient& greeter) {
    print(PRI1, "runSayHello\n");
    for (int i = 0; i < NR_ITERATIONS; ++i) {
        std::string user("world ");
        user += std::to_string(i);
        std::string reply = greeter.SayHello(user);
        print(PRI1, "runSayHello: Greeter received: %s\n", reply.c_str());
    }
}

async_task<void> runSayHelloCo(GreeterClient& greeter) {
    print(PRI1, "runSayHelloCo\n");
    for (int i = 0; i < NR_ITERATIONS; ++i) {
        std::string user("coroutine world: eager - co_await ");
        user += std::to_string(i);
        async_task<std::string> t = greeter.SayHelloCo(user);
        print(PRI1, "runSayHelloCo: co_await t;\n");
        std::string reply = co_await t;
        print(PRI1, "runSayHelloCo: Greeter received: %s\n", reply.c_str());
    }
    co_return;
}

async_task<void> runSayHelloCo2(GreeterClient& greeter) {
    print(PRI1, "runSayHelloCo2\n");
    for (int i = 0; i < NR_ITERATIONS; ++i) {
        std::string user("coroutine world: eager - get_result() ");
        user += std::to_string(i);
        async_task<std::string> t = greeter.SayHelloCo(user);
        runEventQueue(eventQueueThr, 1);
        print(PRI1, "runSayHelloCo2: t.get_result();\n");
        std::string reply = t.get_result();
        print(PRI1, "runSayHelloCo2: Greeter received: %s\n", reply.c_str());
    }
    co_return;
}

async_ltask<void> runSayHelloCoL(GreeterClient& greeter) {
    print(PRI1, "runSayHelloCoL\n");
    for (int i = 0; i < NR_ITERATIONS; ++i) {
        std::string user("coroutine world: lazy - co_await ");
        user += std::to_string(i);
        async_ltask<std::string> t = greeter.SayHelloCoL(user);
        print(PRI1, "runSayHelloCoL: co_await t;\n");
        std::string reply = co_await t;
        print(PRI1, "runSayHelloCoL: Greeter received: %s\n", reply.c_str());
    }
    co_return;
}

async_ltask<void> runSayHelloCoL2(GreeterClient& greeter) {
    print(PRI1, "runSayHelloCoL2\n");
    for (int i = 0; i < NR_ITERATIONS; ++i) {
        std::string user("coroutine world: lazy - get_result() ");
        user += std::to_string(i);
        async_ltask<std::string> t = greeter.SayHelloCoL(user);
        t.start();
        runEventQueue(eventQueueThr, 1);
        print(PRI1, "runSayHelloCoL: t.get_result();\n");
        std::string reply = t.get_result();
        print(PRI1, "runSayHelloCoL: Greeter received: %s\n", reply.c_str());
    }
    co_return;
}

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

  set_priority(0x01);

  GreeterClient greeter(
      grpc::CreateChannel(target_str, grpc::InsecureChannelCredentials()));

  print(PRI1); print(PRI1, "main: runSayHello(greeter);\n");
  runSayHello(greeter);

  print(PRI1); print(PRI1, "main: async_task<void> t1 = runSayHelloCo(greeter);\n");
  async_task<void> t1 = runSayHelloCo(greeter);
  print(PRI1, "main: runEventQueue(eventQueueThr, NR_INTERACTIONS);\n");
  runEventQueue(eventQueueThr, NR_ITERATIONS);
  //print(PRI1, "main: t1.wait();\n");
  //t1.wait();              // No need to call t.wait()

  print(PRI1); print(PRI1, "main: async_task<void> t2 = runSayHelloCo2(greeter);\n");
  async_task<void> t2 = runSayHelloCo2(greeter);
  print(PRI1, "main: t2.wait();\n");
  t2.wait();

  print(PRI1); print(PRI1, "main: async_ltask<void> t3 = runSayHelloCoL(greeter);\n");
  async_ltask<void> t3 = runSayHelloCoL(greeter);
  print(PRI1, "main: t3.start();\n");
  t3.start();
  print(PRI1, "main: runEventQueue(eventQueueThr, NR_INTERACTIONS);\n");
  runEventQueue(eventQueueThr, NR_ITERATIONS);
  //print(PRI1, "main: t3.wait();\n");
  //t3.wait();              // No need to call t.wait()

  print(PRI1); print(PRI1, "main: async_ltask<void> t4 = runSayHelloCoL2(greeter);\n");
  async_ltask<void> t4 = runSayHelloCoL2(greeter);
  print(PRI1, "main: t4.start();\n");
  t4.start();
  print(PRI1, "main: t4.wait();\n");
  t4.wait();

  return 0;
}
