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
 /**
  * @brief Added coroutine implementation. Based on the implementation in greeter_async_client2.cc.
  *
  * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
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

class GreeterClient : public CommService
{
public:
    explicit GreeterClient(std::shared_ptr<Channel> channel)
        : stub_(Greeter::NewStub(channel)) {}

    void AsyncCompleteRpc()
    {
        void* got_tag;
        bool ok = false;

        // Block until the next result is available in the completion queue "cq".
        // The return value of Next should always be checked. This return value
        // tells us whether there is any kind of event or the cq_ is shutting down.
        while (cq_.Next(&got_tag, &ok))
        {
            // Verify that the result from "cq" corresponds, by its tag, our previous
            // request and that the request was completed successfully. Note that "ok"
            // corresponds solely to the request for updates introduced by Finish().
            std::cout << "AsyncCompleteRpc: ok = " << ok << ", got_tag = " << (uint64_t)got_tag << std::endl;

            // See: https://en.cppreference.com/w/cpp/container/deque/erase

            GRPCEvent grpcEvent;
            bool found = false;
            std::deque<GRPCEvent>::iterator it;
            for (it = eventQueue_.begin(); it != eventQueue_.end();)
            {
                grpcEvent = *it;
                if (grpcEvent.p == got_tag)
                {
                    found = true;
                    break;
                }
                ++it;
            }
            if (found)
            {
                grpcEvent.eventHandler();
            }
            else
            {
                std::cout << "Did not find match for " << (uint64_t)got_tag << std::endl;
            }
            
            for (it = eventQueue_.begin(); it != eventQueue_.end();)
            {
                grpcEvent = *it;
                if (grpcEvent.p == got_tag)
                {
                    it = eventQueue_.erase(it);
                    break;
                }
                ++it;
            }
        }
    }

    async_operation<void> start_SayHello(ClientContext* pcontext, HelloRequest& request, HelloReply& reply, Status& status)
    {
        int index = get_free_index();
        async_operation<void> ret{ this, index };
        start_SayHello_impl(index, pcontext, request, reply, status);
        return ret;
    }

    void start_SayHello_impl(int idx, ClientContext* pcontext, HelloRequest& request, HelloReply& reply, Status& status)
    {
        std::unique_ptr<ClientAsyncResponseReader<HelloReply> > rpc(
            stub_->AsyncSayHello(pcontext, request, &cq_));

        std::cout << "start_SayHello_impl:" << request.name() << std::endl;
        uint64_t idx64 = static_cast<uint64_t>(idx);

        rpc->Finish(&reply, &status, (void*)idx64);

        GRPCEvent grpcEvent;

        grpcEvent.p = (void*)idx64;
        grpcEvent.eventHandler =
            [this, idx]()
        {
            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<void>* om_async_operation_t =
                dynamic_cast<async_operation<void>*>(om_async_operation);
            if (om_async_operation_t)
            {
                om_async_operation_t->completed();
            }
        };

        eventQueue_.push_back(grpcEvent);
    }

    async_task<std::string> SayHelloAsync(const std::string& user) 
    {
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

    async_task<void> HelloWorldAsync()
    {
        for (int i = 0; i < 200; i =  i + 4)
        {
            std::string user1("coroutine world " + std::to_string(i));
            async_task<std::string> t1 = SayHelloAsync(user1);
            std::string user2("coroutine world " + std::to_string(i + 1));
            async_task<std::string> t2 = SayHelloAsync(user2);
            std::string user3("coroutine world " + std::to_string(i + 2));
            async_task<std::string> t3 = SayHelloAsync(user3);
            std::string user4("coroutine world " + std::to_string(i + 3));
            async_task<std::string> t4 = SayHelloAsync(user4);
            
            when_all<async_task<std::string>> wa({ &t1, &t2, &t3, &t4 });
            co_await wa;

            std::cout << "Greeter received: " << t1.get_result() << std::endl;
            std::cout << "Greeter received: " << t2.get_result() << std::endl;
            std::cout << "Greeter received: " << t3.get_result() << std::endl;
            std::cout << "Greeter received: " << t4.get_result() << std::endl;
        }
        co_return;
    }

private:
    // Out of the passed in Channel comes the stub, stored here, our view of the
    // server's exposed services.
    std::unique_ptr<Greeter::Stub> stub_;

    // The producer-consumer queue we use to communicate asynchronously with the
    // gRPC runtime.
    CompletionQueue cq_;

    struct GRPCEvent
    {
        void* p;
        std::function<void(void)> eventHandler;
    };

    std::deque<GRPCEvent> eventQueue_;
};

int main(int argc, char** argv) 
{
  // Instantiate the client. It requires a channel, out of which the actual RPCs
  // are created. This channel models a connection to an endpoint (in this case,
  // localhost at port 50051). We indicate that the channel isn't authenticated
  // (use of InsecureChannelCredentials()).
  GreeterClient greeter(grpc::CreateChannel(
                                "localhost:50051", grpc::InsecureChannelCredentials()));

  print(PRI1, "Press control - c to quit\n");
  print(PRI1, "main: greeter.HelloWorldAsync();\n");
  greeter.HelloWorldAsync();
  print(PRI1, "main: greeter.AsyncCompleteRpc();\n");
  greeter.AsyncCompleteRpc();
  print(PRI1, "main: return 0;\n");
  return 0;
}
