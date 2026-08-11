/**
 * @file multiplex_coroutine_client2_lso.cc
 * @brief Added coroutine implementation.
 * Based on the implementation in multiplex_client2.cc.
 * 
 * In contrast to multiplex_coroutine_client2.cc, this implementation ues lsos (lazy-start operation) and lazy-start coroutines.
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
        SayHello_operation_impl(MultiplexClient* greeterClient, ClientContext* context, helloworld::HelloRequest& request, helloworld::HelloReply& reply, Status& status)
            : greeterClient_(greeterClient)
            , context_(context)
            , request_(request)
            , reply_(reply)
            , status_(status) {
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

        void get_result(async_operation_ls_base&) {}

    private:
        MultiplexClient* greeterClient_;
        ClientContext* context_;
        helloworld::HelloRequest& request_;
        helloworld::HelloReply& reply_;
        Status& status_;
    };

    class SayHello_operation : public async_operation_ls<SayHello_operation>
    {
    public:
        SayHello_operation(MultiplexClient* greeterClient, ClientContext* context, helloworld::HelloRequest& request, helloworld::HelloReply& reply, Status& status)
            : m_impl(greeterClient, context, request, reply, status) {
        }

        bool try_start() noexcept { return m_impl.try_start(*this); }
        void get_result() { m_impl.get_result(*this); }

        SayHello_operation_impl m_impl;
    };

    SayHello_operation start_SayHello(ClientContext* context, helloworld::HelloRequest& request, helloworld::HelloReply& reply, Status& status) {
        return SayHello_operation(this, context, request, reply, status);
    }

    // -------------------------------------------------------------------------------------
    
    class GetFeature_operation_impl
    {
    public:
        GetFeature_operation_impl(MultiplexClient* greeterClient, ClientContext* context, routeguide::Point& request, routeguide::Feature& reply, Status& status)
            : greeterClient_(greeterClient)
            , context_(context)
            , request_(request)
            , reply_(reply)
            , status_(status) {
        }

        bool try_start(async_operation_ls_base& operation) noexcept {
            routeguide::RouteGuide::NewStub(greeterClient_->channel_)->async()->GetFeature(context_, &request_, &reply_,
                [this, &operation](Status s) {
                    print(PRI5, "SayHello_operation_impl::try_start: handler\n");
                    status_ = std::move(s);
                    operation.completed();
                });
            return true;
        }

        void get_result(async_operation_ls_base&) {}

    private:
        MultiplexClient* greeterClient_;
        ClientContext* context_;
        routeguide::Point& request_;
        routeguide::Feature& reply_;
        Status& status_;
    };

    class GetFeature_operation : public async_operation_ls<GetFeature_operation>
    {
    public:
        GetFeature_operation(MultiplexClient* greeterClient, ClientContext* context, routeguide::Point& request, routeguide::Feature& reply, Status& status)
            : m_impl(greeterClient, context, request, reply, status) {
        }

        bool try_start() noexcept { return m_impl.try_start(*this); }
        void get_result() { m_impl.get_result(*this); }

        GetFeature_operation_impl m_impl;
    };

    GetFeature_operation start_GetFeature(ClientContext* context, routeguide::Point& request, routeguide::Feature& reply, Status& status) {
        return GetFeature_operation(this, context, request, reply, status);
    }
    // lazy-start operation definition - end

public:
    explicit MultiplexClient(std::shared_ptr<Channel> channel)
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

    async_ltask<std::string> SayHelloCo() {
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

    async_ltask<std::string> GetFeatureCo() {
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

  print(PRI1, "Not using coroutines\n");
  for (int i = 0; i < NR_ITERATIONS; ++i) {
      multiplexClient.SayHello_GetFeature();
  }

  print(PRI1, "\nUsing SayHello_GetFeatureCo\n");
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
