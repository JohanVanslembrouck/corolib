/**
 * @file multiplex_coroutine_client3-when_all.cc
 * @brief Added coroutine implementation.
 * Based on the implementation in multiplex_coroutine_client2.cc.
 * 
 * In this variant start_SayHello and start_GetFeature return async_operation<Status> instead of async_operation<void>.
 * Consequently, there is no need to pass Status as reference argument to start_SayHello and start_GetFeature.
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
#include <corolib/when_all.h>

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

    async_task<void> SayHello_GetFeatureCo_when_all() {
        print(PRI5, "SayHello_GetFeatureCo_when_all - begin\n");        // runs on the original thread 0
        async_task<std::string> t1 = SayHelloCo();
        async_task<std::string> t2 = GetFeatureCo();
        when_all wa(t1, t2);
        print(PRI5, "SayHello_GetFeatureCo_when_all - before co_await wa\n");   // runs on the original thread 0
        co_await wa;
        print(PRI5, "SayHello_GetFeatureCo_when_all - after co_await wa\n");    // runs on another thread 1
        std::cout << t1.get_result();
        std::cout << t2.get_result();
        print(PRI5, "SayHello_GetFeatureCo_when_all - end\n");          // runs on thread 1
        co_return;
    }

    async_task <std::string> SayHelloCo() {
        ClientContext hello_context;
        helloworld::HelloRequest hello_request;
        helloworld::HelloReply hello_response;

        hello_request.set_name("coroutine user");

        Status hello_status = co_await start_SayHello(&hello_context, hello_request, hello_response);

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

    async_operation<Status> start_SayHello(ClientContext* pcontext, helloworld::HelloRequest& request, helloworld::HelloReply& reply) {
        int index = get_free_index();
        async_operation<Status> ret{ this, index };
        helloworld::Greeter::NewStub(channel_)->async()->SayHello(pcontext, &request, &reply,
            [index, this](Status s) {
                print(PRI5, "start_SayHello - completion handler\n");
                Status status = std::move(s);
                completionHandler<Status>(index, status);
            });
        return ret;
    }

    async_task<std::string> GetFeatureCo() {
        ClientContext feature_context;
        routeguide::Point feature_request;
        routeguide::Feature feature_response;

        feature_request.set_latitude(50);
        feature_request.set_longitude(100);

        Status feature_status = co_await start_GetFeature(&feature_context, feature_request, feature_response);

        std::stringstream strstr;
        if (feature_status.ok()) {
            strstr << "Found feature: " << feature_response.name() << std::endl;
        }
        else {
            strstr << "Getting feature failed: " << feature_status.error_message() << std::endl;
        }
        co_return strstr.str();
    }

    async_operation<Status> start_GetFeature(ClientContext* pcontext, routeguide::Point& request, routeguide::Feature& reply) {
        int index = get_free_index();
        async_operation<Status> ret{ this, index };
        routeguide::RouteGuide::NewStub(channel_)->async()->GetFeature(pcontext, &request, &reply,
            [index, this](Status s) {
                print(PRI5, "start_GetFeature - completion handler\n");
                Status status = std::move(s);
                completionHandler<Status>(index, status);
            });
        return ret;
    }

private:
    std::shared_ptr<Channel> channel_;
};

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);
  // Instantiate the client. It requires a channel, out of which the actual RPCs
  // are created. This channel models a connection to an endpoint specified by
  // the argument "--target=" which is the only expected argument.
  std::string target_str = absl::GetFlag(FLAGS_target);

  set_print_level(0x01);        // Use 0x03 to follow the flow in corolib
                                // Use 0x11 to follow the flow in GreeterClient

  GreeterClient greeter(
      grpc::CreateChannel(target_str, grpc::InsecureChannelCredentials()));

  print(PRI1); print(PRI1, "Using SayHello_GetFeatureCo_when_all\n");
  for (int i = 0; i < NR_ITERATIONS; ++i) {
      async_task<void> t = greeter.SayHello_GetFeatureCo_when_all();
      print(PRI2, "Before wait\n");
      t.wait();
      print(PRI2, "After wait\n");

      print(PRI2, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
 
  return 0;
}
