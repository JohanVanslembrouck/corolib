/**
 * @file greeter_coroutine_client2a.cc
 * @brief Added coroutine implementation.
 * Based on the implementation in greeter_async_client.cc and greeter_async_client2.cc.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

/*
 *
 * Copyright 2015 gRPC authors.
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

#include <iostream>
#include <memory>
#include <string>
#include <deque>

#include <grpc/support/log.h>
#include <grpcpp/grpcpp.h>

#include <corolib/print.h>
#include <corolib/commservice.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/when_all.h>

#ifdef BAZEL_BUILD
#include "examples/protos/helloworld.grpc.pb.h"
#else
#include "helloworld.grpc.pb.h"
#endif

using grpc::Channel;
using grpc::ClientAsyncResponseReader;
using grpc::ClientContext;
using grpc::CompletionQueue;
using grpc::Status;
using helloworld::Greeter;
using helloworld::HelloReply;
using helloworld::HelloRequest;

using namespace corolib;

const int NR_ITERATIONS = 100;

class GreeterClient : public CommService
{
public:
    explicit GreeterClient(std::shared_ptr<Channel> channel)
        : stub_(Greeter::NewStub(channel)) {}

    // Not used in this example. Originates from greeter_async_client.cc.
    // 
    // Assembles the client's payload, sends it and presents the response back
    // from the server.
    std::string SayHello(const std::string& user) {
        // >>> To be placed in SayHelloCo - begin
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
        // >>> To be placed in SayHelloCo - end

        // >>> To be used as data member
        // The producer-consumer queue we use to communicate asynchronously with the
        // gRPC runtime.
        CompletionQueue cq;

        // >>> To be placed in start_SayHello_impl - begin
        std::unique_ptr<ClientAsyncResponseReader<HelloReply> > rpc(
            stub_->AsyncSayHello(&context, request, &cq));

        // Request that, upon completion of the RPC, "reply" be updated with the
        // server's response; "status" with the indication of whether the operation
        // was successful. Tag the request with the integer 1.
        rpc->Finish(&reply, &status, (void*)1);
        // >>> To be placed in start_SayHello_impl - end

        void* got_tag;
        bool ok = false;
        // Block until the next result is available in the completion queue "cq".
        // The return value of Next should always be checked. This return value
        // tells us whether there is any kind of event or the cq_ is shutting down.
        GPR_ASSERT(cq.Next(&got_tag, &ok));

        // Verify that the result from "cq" corresponds, by its tag, our previous
        // request.
        GPR_ASSERT(got_tag == (void*)1);
        // ... and that the request was completed successfully. Note that "ok"
        // corresponds solely to the request for updates introduced by Finish().
        GPR_ASSERT(ok);

        // >>> To be placed in SayHelloCo - begin
        // Act upon the status of the actual RPC.
        if (status.ok()) {
            return reply.message();
        }
        else {
            return "RPC failed";
        }
        // >>> To be placed in SayHelloCo - end
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

        co_await start_SayHello(&context, request, reply, status);

        // Act upon the status of the actual RPC.
        if (status.ok()) {
            co_return reply.message();
        }
        else {
            co_return "RPC failed";
        }
    }

    async_operation<void> start_SayHello(ClientContext* pcontext, HelloRequest& request, HelloReply& reply, Status& status) {
        int index = get_free_index();
        async_operation<void> ret{ this, index };
        start_SayHello_impl(index, pcontext, request, reply, status);
        return ret;
    }

    void start_SayHello_impl(int idx, ClientContext* pcontext, HelloRequest& request, HelloReply& reply, Status& status) {
        std::unique_ptr<ClientAsyncResponseReader<HelloReply> > rpc(
            stub_->AsyncSayHello(pcontext, request, &cq_));

        std::cout << "start_SayHello_impl:" << request.name() << std::endl;
        uint64_t idx64 = static_cast<uint64_t>(idx);

        rpc->Finish(&reply, &status, (void*)idx64);

        GRPCEvent grpcEvent;
        grpcEvent.p = (void*)idx64;
        grpcEvent.eventHandler = [this, idx]() { completionHandler_v(idx); };
        eventQueue_.push_back(grpcEvent);
    }

    // Based upon AsyncCompleteRpc in greeter_async_client2.cc.
    void AsyncCompleteRpc() {
        void* got_tag;
        bool ok = false;

        // Block until the next result is available in the completion queue "cq".
        // The return value of Next should always be checked. This return value
        // tells us whether there is any kind of event or the cq_ is shutting down.
        while (!done_ && cq_.Next(&got_tag, &ok)) {
            // Verify that the result from "cq" corresponds, by its tag, our previous
            // request and that the request was completed successfully. Note that "ok"
            // corresponds solely to the request for updates introduced by Finish().
            std::cout << "AsyncCompleteRpc: ok = " << ok << ", got_tag = " << (uint64_t)got_tag << std::endl;

            // See: https://en.cppreference.com/w/cpp/container/deque/erase

            GRPCEvent grpcEvent;
            bool found = false;
            std::deque<GRPCEvent>::iterator it;
            for (it = eventQueue_.begin(); it != eventQueue_.end();) {
                grpcEvent = *it;
                if (grpcEvent.p == got_tag) {
                    found = true;
                    break;
                }
                ++it;
            }
            if (found) {
                grpcEvent.eventHandler();
            }
            else {
                std::cout << "Did not find match for " << (uint64_t)got_tag << std::endl;
            }

            for (it = eventQueue_.begin(); it != eventQueue_.end();) {
                grpcEvent = *it;
                if (grpcEvent.p == got_tag) {
                    it = eventQueue_.erase(it);
                    break;
                }
                ++it;
            }
        }
    }

    void setDone() {
        done_ = true;
    }

private:

    // Out of the passed in Channel comes the stub, stored here, our view of the
    // server's exposed services.
    std::unique_ptr<Greeter::Stub> stub_;

    // The producer-consumer queue we use to communicate asynchronously with the
    // gRPC runtime.
    CompletionQueue cq_;

    // Added for the use of corolib - begin
    struct GRPCEvent {
        void* p;
        std::function<void(void)> eventHandler;
    };

    std::deque<GRPCEvent> eventQueue_;

    bool done_ = false;
    // Added for the use of corolib - end
};

// Top level coroutine. Added because main() cannot be a coroutine.
async_task<void> runSayHelloCo(GreeterClient& greeter) {
    for (int i = 0; i < NR_ITERATIONS; i = i + 4) {
        std::string user1("coroutine world " + std::to_string(i));
        async_task<std::string> t1 = greeter.SayHelloCo(user1);
        std::string user2("coroutine world " + std::to_string(i + 1));
        async_task<std::string> t2 = greeter.SayHelloCo(user2);
        std::string user3("coroutine world " + std::to_string(i + 2));
        async_task<std::string> t3 = greeter.SayHelloCo(user3);
        std::string user4("coroutine world " + std::to_string(i + 3));
        async_task<std::string> t4 = greeter.SayHelloCo(user4);

        when_all wa({ &t1, &t2, &t3, &t4 });
        co_await wa;

        std::cout << "Greeter received: " << t1.get_result() << std::endl;
        std::cout << "Greeter received: " << t2.get_result() << std::endl;
        std::cout << "Greeter received: " << t3.get_result() << std::endl;
        std::cout << "Greeter received: " << t4.get_result() << std::endl;
    }
    greeter.setDone();
    co_return;
}

int main(int argc, char** argv)  {
  // Instantiate the client. It requires a channel, out of which the actual RPCs
  // are created. This channel models a connection to an endpoint (in this case,
  // localhost at port 50051). We indicate that the channel isn't authenticated
  // (use of InsecureChannelCredentials()).
  GreeterClient greeter(grpc::CreateChannel(
                                "localhost:50051", grpc::InsecureChannelCredentials()));

  print(PRI1, "main: async_task<void> t = runSayHelloCo(greeter);\n");
  async_task<void> t = runSayHelloCo(greeter);
  print(PRI1, "main: greeter.AsyncCompleteRpc();\n");
  greeter.AsyncCompleteRpc();
  print(PRI1, "main: t.wait();\n");
  t.wait();

  return 0;
}
