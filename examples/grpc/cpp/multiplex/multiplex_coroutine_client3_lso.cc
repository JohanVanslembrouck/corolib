/**
 * @file multiplex_coroutine_client3_lso.cc
 * @brief Added coroutine implementation.
 * Based on the implementation in multiplex_coroutine_client3.cc.
 * 
 * In this variant start_SayHello and start_GetFeature return async_operation<Status> instead of async_operation<void>.
 * Consequently, there is no need to pass Status as reference argument to start_SayHello and start_GetFeature.
 * Notice that the real reply is still passed via a reference argument.
 * 
 * In contrast to multiplex_coroutine_client3.cc, this implementation ues lsos (lazy-start operation) and lazy-start coroutines.
 * 
 * @author Johan Vanslembrouck
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

class MultiplexClient : public CommService
{
private:
    // lazy-start operation definition - begin
    class SayHello_operation_impl
    {
    public:
        SayHello_operation_impl(MultiplexClient* greeterClient, ClientContext* context, helloworld::HelloRequest& request, helloworld::HelloReply& reply)
            : greeterClient_(greeterClient)
            , context_(context)
            , request_(request)
            , reply_(reply) {
        }

        bool try_start(async_operation_ls_base& operation) noexcept {
            helloworld::Greeter::NewStub(greeterClient_->channel_)->async()->SayHello(context_, &request_, &reply_,
                [this, &operation](Status s) {
                    print(PRI5, "SayHello_operation_impl::try_start: handler\n");
                    status_ = std::move(s);
                    operation.completed();
                });
            return true;
        }

        Status get_result(async_operation_ls_base&) {
            return status_;
        }

    private:
        MultiplexClient* greeterClient_;
        ClientContext* context_;
        helloworld::HelloRequest& request_;
        helloworld::HelloReply& reply_;
        Status status_;
    };

    class SayHello_operation : public async_operation_ls<SayHello_operation>
    {
    public:
        SayHello_operation(MultiplexClient* greeterClient, ClientContext* context, helloworld::HelloRequest& request, helloworld::HelloReply& reply)
            : m_impl(greeterClient, context, request, reply) {
        }

        bool try_start() noexcept { return m_impl.try_start(*this); }
        Status get_result() { return m_impl.get_result(*this); }

        SayHello_operation_impl m_impl;
    };

    SayHello_operation start_SayHello(ClientContext* context, helloworld::HelloRequest& request, helloworld::HelloReply& reply) {
        return SayHello_operation(this, context, request, reply);
    }

    // -------------------------------------------------------------------------------------

    class GetFeature_operation_impl
    {
    public:
        GetFeature_operation_impl(MultiplexClient* greeterClient, ClientContext* context, routeguide::Point& request, routeguide::Feature& reply)
            : greeterClient_(greeterClient)
            , context_(context)
            , request_(request)
            , reply_(reply) {
        }

        bool try_start(async_operation_ls_base& operation) noexcept {
            routeguide::RouteGuide::NewStub(greeterClient_->channel_)->async()->GetFeature(context_, &request_, &reply_,
                [this, &operation](Status s) {
                    print(PRI5, "GetFeature_operation_impl::trystart - handler\n");
                    status_ = std::move(s);
                    operation.completed();
                });

            return true;
        }

        Status get_result(async_operation_ls_base&) {
            return status_;
        }

    private:
        MultiplexClient* greeterClient_;
        ClientContext* context_;
        routeguide::Point& request_;
        routeguide::Feature& reply_;
        Status status_;
    };

    class GetFeature_operation : public async_operation_ls<GetFeature_operation>
    {
    public:
        GetFeature_operation(MultiplexClient* greeterClient, ClientContext* context, routeguide::Point& request, routeguide::Feature& reply)
            : m_impl(greeterClient, context, request, reply) {
        }

        bool try_start() noexcept { return m_impl.try_start(*this); }
        Status get_result() { return m_impl.get_result(*this); }

        GetFeature_operation_impl m_impl;
    };

    GetFeature_operation start_GetFeature(ClientContext* context, routeguide::Point& request, routeguide::Feature& reply) {
        return GetFeature_operation(this, context, request, reply);
    }
    // lazy-start operation definition - end

public:
    explicit MultiplexClient(std::shared_ptr<Channel> channel)
        : channel_(channel)
    {}

    async_ltask<void> SayHello_GetFeatureCo() {
        print(PRI5, "SayHello_GetFeatureCo - begin\n");    // runs on the original thread
        async_ltask<std::string> t1 = SayHelloCo();
        async_ltask<std::string> t2 = GetFeatureCo();
        std::string helloReply = co_await t1;
        print(PRI5, "SayHello_GetFeatureCo - before co_await t2\n");   // runs on another thread 1
        std::string featureReply = co_await t2;
        print(PRI5, "SayHello_GetFeatureCo - after co_await t2\n");    // runs on yet another thread 2
        std::cout << helloReply;
        std::cout << featureReply;
        print(PRI5, "SayHello_GetFeatureCo - end\n");      // runs on thread 2
        co_return;
    }

    async_ltask <std::string> SayHelloCo() {
        ClientContext hello_context;
        helloworld::HelloRequest hello_request;
        helloworld::HelloReply hello_response;

        hello_request.set_name("coroutine user");

        print(PRI5, "SayHelloCo - before co_await\n");      // runs on original thread
        Status hello_status = co_await start_SayHello(&hello_context, hello_request, hello_response);
        print(PRI5, "SayHelloCo - after co_await\n");       // runs on different thread

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

    async_ltask<std::string> GetFeatureCo() {
        ClientContext feature_context;
        routeguide::Point feature_request;
        routeguide::Feature feature_response;

        feature_request.set_latitude(50);
        feature_request.set_longitude(100);

        print(PRI5, "GetFeatureCo - before co_await\n");    // runs on original thread
        Status feature_status = co_await start_GetFeature(&feature_context, feature_request, feature_response);
        print(PRI5, "GetFeatureCo - after co_await\n");     // runs on different thread

        std::stringstream strstr;
        if (feature_status.ok()) {
            strstr << "Found feature: " << feature_response.name() << std::endl;
        }
        else {
            strstr << "Getting feature failed: " << feature_status.error_message() << std::endl;
        }
        co_return strstr.str();
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
                                // Use 0x11 to follow the flow in MultiplexClient

  MultiplexClient multiplexClient(
      grpc::CreateChannel(target_str, grpc::InsecureChannelCredentials()));

  print(PRI1); print(PRI1, "Using SayHello_GetFeatureCo\n");
  for (int i = 0; i < NR_ITERATIONS; ++i) {
      async_ltask<void> t = multiplexClient.SayHello_GetFeatureCo();
      print(PRI2, "Before start\n");
      t.start();    // An async_ltask object must be started explicitly.
      print(PRI2, "Before wait\n");
      t.wait();
      print(PRI2, "After wait\n");

      print(PRI2, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
 
  return 0;
}
