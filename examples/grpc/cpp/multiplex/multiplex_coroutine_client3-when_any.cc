/**
 * @file multiplex_coroutine_client3-when_any.cc
 * @brief Added coroutine implementation.
 * Based on the implementation in multiplex_coroutine_client2.cc.
 * 
 * In this variant start_SayHello and start_GetFeature return async_operation<Status> instead of async_operation<void>.
 * Consequently, there is no need to pass Status as reference argument to start_SayHello and start_GetFeature.
 * 
 * Notice that start_SayHello and start_GetFeature have a more complicated implementation
 * than their counterparts in multiplex_coroutine_client3.cc and multiplex_coroutine_client3-when_all.cc
 * to deal with concurrency issues unique to when_any.
 * To see the effects of the original implementation, run multiplex_coroutine_client3-all,
 * or compile-in the last part of main() by changing '#if 0' into '#if 1'
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
#include <corolib/when_any.h>

#if USE_LAZY_START_TASKS
#define task async_ltask
#else
#define task async_task
#endif

ABSL_FLAG(std::string, target, "localhost:50051", "Server address");

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;

using namespace corolib;

const int NR_ITERATIONS = 100;

class MultiplexClient : public CommService
{
private:
#if !USE_LAZY_START_OPS
    // eager-start operation definition - begin
#if 0
    // negative diff for coroutine count if we use the "unprotected" version of the functions
    async_operation<Status> start_SayHello(ClientContext* context, helloworld::HelloRequest& request, helloworld::HelloReply& reply) {
        int index = get_free_index();
        async_operation<Status> ret{ this, index };
        helloworld::Greeter::NewStub(channel_)->async()->SayHello(context, &request, &reply,
            [index, this](Status s) {
                print(PRI5, "start_SayHello - completion handler\n");
                Status status = std::move(s);
                completionHandler<Status>(index, status);
            });
        return ret;
    }

    async_operation<Status> start_GetFeature(ClientContext* context, routeguide::Point& request, routeguide::Feature& reply) {
        int index = get_free_index();
        async_operation<Status> ret{ this, index };
        routeguide::RouteGuide::NewStub(channel_)->async()->GetFeature(context, &request, &reply,
            [index, this](Status s) {
                print(PRI5, "start_GetFeature - completion handler\n");
                Status status = std::move(s);
                completionHandler<Status>(index, status);
            });
        return ret;
    }
#else
    async_operation<Status> start_SayHello(ClientContext* context, helloworld::HelloRequest& request, helloworld::HelloReply& reply) {
        int index = get_free_index();
        async_operation<Status> ret{ this, index };
        helloworld::Greeter::NewStub(channel_)->async()->SayHello(context, &request, &reply,
            [index, this](Status s) {
                print(PRI5, "start_SayHello - completion handler\n");
                Status status = std::move(s);
                async_operation_base* om_async_operation = get_async_operation(index);
                async_operation<Status>* om_async_operation_t =
                    static_cast<async_operation<Status>*>(om_async_operation);
                if (om_async_operation_t) {
                    om_async_operation_t->set_result(status);
                    if (m_use_mutex) {
                        std::lock_guard<std::mutex> guard(m_mutex);
                        om_async_operation_t->completed();
                    }
                    else
                        om_async_operation_t->completed();
                }
            });
        return ret;
    }

    async_operation<Status> start_GetFeature(ClientContext* context, routeguide::Point& request, routeguide::Feature& reply) {
        int index = get_free_index();
        async_operation<Status> ret{ this, index };
        routeguide::RouteGuide::NewStub(channel_)->async()->GetFeature(context, &request, &reply,
            [index, this](Status s) {
                print(PRI5, "start_GetFeature - completion handler\n");
                Status status = std::move(s);
                async_operation_base* om_async_operation = get_async_operation(index);
                async_operation<Status>* om_async_operation_t =
                    static_cast<async_operation<Status>*>(om_async_operation);
                if (om_async_operation_t) {
                    om_async_operation_t->set_result(status);
                    if (m_use_mutex) {
                        std::lock_guard<std::mutex> guard(m_mutex);
                        om_async_operation_t->completed();
                    }
                    else
                        om_async_operation_t->completed();
                }
            });
        return ret;
    }
#endif
    // eager-start operation definition - end
#else
    // lazy-start operation definition - begin
    class SayHello_operation_impl
    {
    public:
        SayHello_operation_impl(MultiplexClient* multiplexClient, ClientContext* context, helloworld::HelloRequest& request, helloworld::HelloReply& reply)
            : multiplexClient_(multiplexClient)
            , context_(context)
            , request_(request)
            , reply_(reply) {
        }

        bool try_start(async_operation_ls_base& operation) noexcept {
            helloworld::Greeter::NewStub(multiplexClient_->channel_)->async()->SayHello(context_, &request_, &reply_,
                [this, &operation](Status s) {
                    print(PRI5, "SayHello_operation_impl::try_start: handler\n");
                    status_ = std::move(s);
                    if (multiplexClient_->m_use_mutex) {
                        std::lock_guard<std::mutex> guard(multiplexClient_->m_mutex);
                        operation.completed();
                    }
                    else
                        operation.completed();
                });
            return true;
        }

        Status get_result(async_operation_ls_base&) {
            return status_;
        }

    private:
        MultiplexClient* multiplexClient_;
        ClientContext* context_;
        helloworld::HelloRequest& request_;
        helloworld::HelloReply& reply_;
        Status status_;
    };

    class SayHello_operation : public async_operation_ls<SayHello_operation>
    {
    public:
        SayHello_operation(MultiplexClient* multiplexClient, ClientContext* context, helloworld::HelloRequest& request, helloworld::HelloReply& reply)
            : m_impl(multiplexClient, context, request, reply) {
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
        GetFeature_operation_impl(MultiplexClient* multiplexClient, ClientContext* context, routeguide::Point& request, routeguide::Feature& reply)
            : multiplexClient_(multiplexClient)
            , context_(context)
            , request_(request)
            , reply_(reply) {
        }

        bool try_start(async_operation_ls_base& operation) noexcept {
            routeguide::RouteGuide::NewStub(multiplexClient_->channel_)->async()->GetFeature(context_, &request_, &reply_,
                [this, &operation](Status s) {
                    print(PRI5, "GetFeature_operation_impl::try_start - handler\n");
                    status_ = std::move(s);
                    if (multiplexClient_->m_use_mutex) {
                        std::lock_guard<std::mutex> guard(multiplexClient_->m_mutex);
                        operation.completed();
                    }
                    else
                        operation.completed();
                });

            return true;
        }

        Status get_result(async_operation_ls_base&) {
            return status_;
        }

    private:
        MultiplexClient* multiplexClient_;
        ClientContext* context_;
        routeguide::Point& request_;
        routeguide::Feature& reply_;
        Status status_;
    };

    class GetFeature_operation : public async_operation_ls<GetFeature_operation>
    {
    public:
        GetFeature_operation(MultiplexClient* multiplexClient, ClientContext* context, routeguide::Point& request, routeguide::Feature& reply)
            : m_impl(multiplexClient, context, request, reply) {
        }

        bool try_start() noexcept { return m_impl.try_start(*this); }
        Status get_result() { return m_impl.get_result(*this); }

        GetFeature_operation_impl m_impl;
    };

    GetFeature_operation start_GetFeature(ClientContext* context, routeguide::Point& request, routeguide::Feature& reply) {
        return GetFeature_operation(this, context, request, reply);
    }
    // lazy-start operation definition - end
#endif

public:
    explicit MultiplexClient(std::shared_ptr<Channel> channel)
        : channel_(channel)
    {}

    task<void> SayHello_GetFeatureCo_when_any() {
        print(PRI5, "SayHello_GetFeatureCo_when_any - begin\n");    // runs on the original thread 0
        task<std::string> t1 = SayHelloCo();
        task<std::string> t2 = GetFeatureCo();
        when_any wa(t1, t2);
        print(PRI5, "SayHello_GetFeatureCo_when_any - before loop\n");    // runs on the original thread 0
        for (int i = 0; i < 2; ++i) {
            int s = co_await wa;
            print(PRI5, "SayHello_GetFeatureCo_when_any - after co_await wa\n");    // runs on another thread i, one thread per iteration
            switch (s) {
                case 0: std::cout << t1.get_result(); break;
                case 1: std::cout << t2.get_result(); break;
                default: std::cout << "SayHello_GetFeatureCo_when_any: Unexpected reply " << s << std::endl;
            }
        }
        print(PRI5, "SayHello_GetFeatureCo_when_any - end\n");      // runs on thread i
        co_return;
    }

    task <std::string> SayHelloCo() {
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

    task<std::string> GetFeatureCo() {
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

    void set_use_mutex(bool use_mutex) {
        m_use_mutex = use_mutex;
    }

private:
    std::shared_ptr<Channel> channel_;
    bool m_use_mutex = true;
    std::mutex m_mutex;
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

  print(PRI1); print(PRI1, "Using SayHello_GetFeatureCo_when_any\n");
  for (int i = 0; i < NR_ITERATIONS; ++i) {
      task<void> t = multiplexClient.SayHello_GetFeatureCo_when_any();
      t.start();
      print(PRI2, "Before wait: i = %d\n", i);
      t.wait();
      print(PRI2, "After wait: i = %d\n", i);

      print(PRI2, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
#if 0
  multiplexClient.set_use_mutex(false);

  print(PRI1, "Using SayHello_GetFeatureCo_when_any\n");
  for (int i = 0; i < NR_ITERATIONS; ++i) {
      task<void> t = multiplexClient.SayHello_GetFeatureCo_when_any();
      t.start();
      print(PRI2, "Before wait: i = %d\n", i);
      t.wait();
      print(PRI2, "After wait: i = %d\n", i);

      print(PRI2, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
#endif
  return 0;
}
