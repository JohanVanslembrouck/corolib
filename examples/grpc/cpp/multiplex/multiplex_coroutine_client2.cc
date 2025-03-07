/**
 * @file multiplex_coroutine_client3.cc
 * @brief Added coroutine implementation.
 * Based on the implementation in multiplex_client2.cc.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

/*
 *
 * Copyright 2023 gRPC authors.
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
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"

#include <grpcpp/grpcpp.h>

#ifdef BAZEL_BUILD
#include "examples/protos/helloworld.grpc.pb.h"
#include "examples/protos/route_guide.grpc.pb.h"
#else
#include "helloworld.grpc.pb.h"
#include "route_guide.grpc.pb.h"
#endif

#include <corolib/print.h>
#include <corolib/commservice.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

ABSL_FLAG(std::string, target, "localhost:50051", "Server address");

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

using namespace corolib;

const int NR_ITERATIONS = 100;

class GreeterClient : public CommService
{
public:
    explicit GreeterClient(std::shared_ptr<Channel> channel)
        : channel_(channel)
    {}

    // Assembles the client's payload, sends it and presents the response back
    // from the server.
    void SayHello_GetFeature() {
        std::mutex mu;
        std::condition_variable cv;
        int done_count = 0;

        // Callbacks will be called on background threads
        std::unique_lock<std::mutex> lock(mu);

        ClientContext hello_context;
        helloworld::HelloRequest hello_request;
        helloworld::HelloReply hello_response;
        Status hello_status;

        ClientContext feature_context;
        routeguide::Point feature_request;
        routeguide::Feature feature_response;
        Status feature_status;

        // Request to a Greeter service
        hello_request.set_name("user");
        helloworld::Greeter::NewStub(channel_)->async()->SayHello(
            &hello_context, &hello_request, &hello_response,
            [&](Status status) {
                std::lock_guard<std::mutex> lock(mu);
                done_count++;
                hello_status = std::move(status);
                cv.notify_all();
            });

        // Request to a RouteGuide service
        feature_request.set_latitude(50);
        feature_request.set_longitude(100);
        routeguide::RouteGuide::NewStub(channel_)->async()->GetFeature(
            &feature_context, &feature_request, &feature_response,
            [&](Status status) {
                std::lock_guard<std::mutex> lock(mu);
                done_count++;
                feature_status = std::move(status);
                cv.notify_all();
            });
        // Wait for both requests to finish
        cv.wait(lock, [&]() { return done_count == 2; });
        if (hello_status.ok()) {
            std::cout << "Greeter received: " << hello_response.message() << std::endl;
        }
        else {
            std::cerr << "Greeter failed: " << hello_status.error_message()
                << std::endl;
        }
        if (feature_status.ok()) {
            std::cout << "Found feature: " << feature_response.name() << std::endl;
        }
        else {
            std::cerr << "Getting feature failed: " << feature_status.error_message()
                << std::endl;
        }
    }

    async_task<void> SayHello_GetFeatureCo() {
        async_task<std::string> t1 = SayHelloAsync();
        async_task<std::string> t2 = GetFeatureAsync();
        std::string helloReply = co_await t1;
        std::string featureReply = co_await t2;
        std::cout << helloReply;
        std::cout << featureReply;
        co_return;
    }

    async_task<std::string> SayHelloAsync() {
        ClientContext hello_context;
        helloworld::HelloRequest hello_request;
        helloworld::HelloReply hello_response;
        Status hello_status;

        hello_request.set_name("coroutine user");

        co_await start_SayHello(&hello_context, hello_request, hello_response, hello_status);

        std::stringstream strstr;
        // Act upon the status of the actual RPC.
        if (hello_status.ok()) {
            strstr << "Greeter received: " << hello_response.message() << std::endl;
        }
        else {
            strstr << "Greeter failed: " << hello_status.error_message() << std::endl;
        }
        co_return strstr.str();
    }

    async_operation<void> start_SayHello(ClientContext* pcontext, helloworld::HelloRequest& request, helloworld::HelloReply& reply, Status& status) {
        int index = get_free_index();
        async_operation<void> ret{ this, index };
        helloworld::Greeter::NewStub(channel_)->async()->SayHello(pcontext, &request, &reply,
            [&status, index, this](Status s) {
                print(PRI5, "start_SayHello - completion handler\n");
                status = std::move(s);
                completionHandler_v(index);
            });
        return ret;
    }

    async_task<std::string> GetFeatureAsync() {
        ClientContext feature_context;
        routeguide::Point feature_request;
        routeguide::Feature feature_response;
        Status feature_status;

        feature_request.set_latitude(50);
        feature_request.set_longitude(100);

        co_await start_GetFeature(&feature_context, feature_request, feature_response, feature_status);

        std::stringstream strstr;
        if (feature_status.ok()) {
            strstr << "Found feature: " << feature_response.name() << std::endl;
        }
        else {
            strstr << "Getting feature failed: " << feature_status.error_message() << std::endl;
        }
        co_return strstr.str();
    }

    async_operation<void> start_GetFeature(ClientContext* pcontext, routeguide::Point& request, routeguide::Feature& reply, Status& status) {
        int index = get_free_index();
        async_operation<void> ret{ this, index };
        routeguide::RouteGuide::NewStub(channel_)->async()->GetFeature(pcontext, &request, &reply,
            [&status, index, this](Status s) {
                print(PRI5, "start_GetFeature - completion handler\n");
                status = std::move(s);
                completionHandler_v(index);
            });
        return ret;
    }

private:
    std::shared_ptr<Channel> channel_;
};

int main(int argc, char** argv) {
  set_print_level(0x01);

  absl::ParseCommandLine(argc, argv);
  // Instantiate the client. It requires a channel, out of which the actual RPCs
  // are created. This channel models a connection to an endpoint specified by
  // the argument "--target=" which is the only expected argument.
  std::string target_str = absl::GetFlag(FLAGS_target);

  set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

  GreeterClient greeter(
      grpc::CreateChannel(target_str, grpc::InsecureChannelCredentials()));

  print(PRI1, "Not using coroutines\n");
  for (int i = 0; i < NR_ITERATIONS; ++i) {
      greeter.SayHello_GetFeature();
  }

  print(PRI1, "Using coroutines\n");
  for (int i = 0; i < NR_ITERATIONS; ++i) {
      async_task<void> t = greeter.SayHello_GetFeatureCo();
      print(PRI2, "Before wait\n");
      t.wait();
      print(PRI2, "After wait\n");

      print(PRI2, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}
