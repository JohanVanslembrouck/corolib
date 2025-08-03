 /**
  * @file multigreeter_coroutine_client.cc
  * @brief Added coroutine implementation. Based on the implementation in multigreeter_client.cc.
  * Original source: https://groups.google.com/g/grpc-io/c/2wyoDZT5eao
  * 
  * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
  */

#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <grpcpp/grpcpp.h>

// added for using corolib - begin
#include <corolib/print.h>
#include <corolib/commservice.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
// added for using corolib - end

#ifdef BAZEL_BUILD
#include "examples/protos/hellostreamingworld.grpc.pb.h"
#else
#include "hellostreamingworld.grpc.pb.h"
#endif

using grpc::Channel;
using grpc::ClientContext;
using grpc::CompletionQueue;
using grpc::Status;

using grpc::ClientAsyncReaderInterface;

using hellostreamingworld::MultiGreeter;
using hellostreamingworld::HelloReply;
using hellostreamingworld::HelloRequest;

#define USE_COROUTINES 1

// Added for using corolib
using namespace corolib;

class GreeterClient : public CommService {      // Added CommService as base class
public:
    explicit GreeterClient(std::shared_ptr<Channel> channel)
        : stub_(MultiGreeter::NewStub(channel)) {}

    // Assembles the client's payload and sends it to the server.
    void SayHello(const std::string& user) {
        HelloRequest request;
        // Data we are sending to the server.
        request.set_name(user);

        // Call object to store rpc data
        AsyncClientCall* call = new AsyncClientCall(this);      // JVS: allow terminating the program automatically

        // stub_->AsyncSayHello() performs the RPC call, returning an instance to
        // store in "call". Because we are using the asynchronous API, we need to
        // hold on to the "call" instance in order to get updates on the ongoing RPC.
        call->response_reader = stub_->AsyncsayHello(&call->context, request, &cq_, (void*)call);
    }

    // Top level coroutine. Added because main() cannot be a coroutine.
    async_task<void> runSayHelloCo(const std::string& user) {
        co_await SayHelloCo(user);
        done_ = true;           // JVS: allow terminating the program automatically
        co_return;
    }

    // Coroutine version of SayHello
    async_task<void> SayHelloCo(const std::string& user) {
        print(PRI1, "SayHelloCo: begin\n");
        HelloRequest request;
        // Data we are sending to the server.
        request.set_name(user);

        co_await start_SayHello(request);
        print(PRI1, "SayHelloCo: end\n");
        co_return;
    }

    async_operation<void> start_SayHello(HelloRequest& request) {
        int index = get_free_index();
        async_operation<void> ret{ this, index };
        start_SayHello_impl(index, request);
        return ret;
    }

    void start_SayHello_impl(int idx, HelloRequest& request) {
        // Call object to store rpc data
        AsyncClientCall* call = new AsyncClientCall(this);

        // stub_->AsyncSayHello() performs the RPC call, returning an instance to
        // store in "call". Because we are using the asynchronous API, we need to
        // hold on to the "call" instance in order to get updates on the ongoing RPC.
        call->response_reader = stub_->AsyncsayHello(&call->context, request, &cq_, (void*)call);

#if USE_COROUTINES
        call->completionHandler_ =
            [this, idx](Status) {
                print(PRI1, "completionHandler\n");
                // completionHandler_v is defined in commservice.h
                this->completionHandler_v(idx);
            };
#endif
    }

    // Used from main
    // Loop while listening for completed responses.
    // Prints out the response from the server.
    void AsyncCompleteRpc() {
        void* got_tag;
        bool ok = false;

        // Block until the next result is available in the completion queue "cq".
        while (!done_ && cq_.Next(&got_tag, &ok)) {         // JVS: allow terminating the program automatically
            // The tag in this example is the memory location of the call object
            ResponseHandler* responseHandler = static_cast<ResponseHandler*>(got_tag);
            print(PRI1, "Tag received: %p\n", responseHandler);

            // Verify that the request was completed successfully. Note that "ok"
            // corresponds solely to the request for updates introduced by Finish().
            print(PRI1, "Next returned: %d\n", ok);
            responseHandler->HandleResponse(ok);
        }
    }

private:

    bool done_ = false;     // JVS: allow terminating the program automatically

    class ResponseHandler {
    public:
        virtual bool HandleResponse(bool eventStatus) = 0;
    };

    // struct for keeping state and data information
    class AsyncClientCall: public ResponseHandler {
        enum CallStatus {CREATE, PROCESS, PROCESSED, FINISH};
        CallStatus callStatus_;
    public:

        AsyncClientCall(GreeterClient* gc) : callStatus_(CREATE), gc_(gc) {}    // JVS: allow terminating the program automatically

        virtual ~AsyncClientCall() {}   // JVS: to avoid g++ complaining about the absence of a virtual destructor

        // Container for the data we expect from the server.
        HelloReply reply;
        // Context for the client. It could be used to convey extra information to
        // the server and/or tweak certain RPC behaviors.
        ClientContext context;

        GreeterClient* gc_;     // JVS: allow terminating the program automatically

        // Storage for the status of the RPC upon completion.
        Status status;
#if USE_COROUTINES
        std::function<void(Status)> completionHandler_;
#endif
        std::unique_ptr<ClientAsyncReaderInterface<HelloReply>> response_reader;

        bool HandleResponse(bool responseStatus) override {
            switch (callStatus_) {
            case CREATE:
                if (responseStatus) {
                    response_reader->Read(&reply, (void*)this);
                    callStatus_ = PROCESS;
                } else {
                    response_reader->Finish(&status, (void*)this);
                    callStatus_ = FINISH;
                }
                break;
            case PROCESS:
                if (responseStatus) {
                    print(PRI1, "Greeter received: %p : %s\n", this, reply.message().c_str());
                    response_reader->Read(&reply, (void*)this);
                } else {
                    response_reader->Finish(&status, (void*)this);
                    callStatus_ = FINISH;
                }
                break;
            case FINISH:
                if (status.ok()) {
                    print(PRI1, "Server Response Completed: %p CallData: %p\n", this, this);
                }
                else {
                    print(PRI1, "RPC failed\n");
                }
#if USE_COROUTINES
                completionHandler_(status);
#else
                gc_->done_ = true;      // JVS: allow terminating the program automatically
#endif
                delete this;
                break;
            default:    // JVS: to avoid g++ complaining about non-covered cases
                ;
            }
            return true;    // JVS
        }
    };

    // Out of the passed in Channel comes the stub, stored here, our view of the
    // server's exposed services.
    std::unique_ptr<MultiGreeter::Stub> stub_;

    // The producer-consumer queue we use to communicate asynchronously with the
    // gRPC runtime.
    CompletionQueue cq_;
};

// main function is very close to the main function in greeter_async_client2.cc
int main(int argc, char** argv) {
    // Instantiate the client. It requires a channel, out of which the actual RPCs
    // are created. This channel models a connection to an endpoint (in this case,
    // localhost at port 50051). We indicate that the channel isn't authenticated
    // (use of InsecureChannelCredentials()).
    GreeterClient greeter(grpc::CreateChannel(
                              "localhost:50051", grpc::InsecureChannelCredentials()));

    // Spawn reader thread that loops indefinitely
    std::thread thread_ = std::thread(&GreeterClient::AsyncCompleteRpc, &greeter);
#if !USE_COROUTINES
    std::string user("world");
    greeter.SayHello(user);  // The actual RPC call!
#else
    std::string user("coroutine world");
    async_task<void> t = greeter.runSayHelloCo(user);
    t.wait();
#endif

    //std::cout << "Press control-c to quit" << std::endl << std::endl;
    thread_.join();

    return 0;
}
