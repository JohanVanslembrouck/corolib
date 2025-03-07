/**
 * @file multigreeter_client.cpp
 * @brief
 * Original source: https://groups.google.com/g/grpc-io/c/2wyoDZT5eao
 * @author: Kuldeep Melligeri
 */

#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <grpcpp/grpcpp.h>

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

class GreeterClient {
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
        call->response_reader = stub_->AsyncsayHello(&call->context, request, &cq_, (void *)call);
    }

    // Loop while listening for completed responses.
    // Prints out the response from the server.
    void AsyncCompleteRpc() {
        void* got_tag;
        bool ok = false;

        // Block until the next result is available in the completion queue "cq".
        while (!done_ && cq_.Next(&got_tag, &ok)) {         // JVS: allow terminating the program automatically
            // The tag in this example is the memory location of the call object
            ResponseHandler* responseHandler = static_cast<ResponseHandler*>(got_tag);
            std::cout << "Tag received: " << responseHandler << std::endl;

            // Verify that the request was completed successfully. Note that "ok"
            // corresponds solely to the request for updates introduced by Finish().
            std::cout << "Next returned: " << ok << std::endl;
            responseHandler->HandleResponse(ok);
        }
    }

    bool done_ = false;     // JVS: allow terminating the program automatically

private:

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
                    std::cout << "Greeter received: " << this << " : " << reply.message() << std::endl;
                    response_reader->Read(&reply, (void*)this);
                } else {
                    response_reader->Finish(&status, (void*)this);
                    callStatus_ = FINISH;
                }
                break;
            case FINISH:
                if (status.ok()) {

                    std::cout << "Server Response Completed: " << this << " CallData: " << this << std::endl;
                    gc_->done_ = true;      // JVS: allow terminating the program automatically
                }
                else {
                    std::cout << "RPC failed" << std::endl;
                }
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

int main(int argc, char** argv) {
    // Instantiate the client. It requires a channel, out of which the actual RPCs
    // are created. This channel models a connection to an endpoint (in this case,
    // localhost at port 50051). We indicate that the channel isn't authenticated
    // (use of InsecureChannelCredentials()).
    GreeterClient greeter(grpc::CreateChannel(
                              "localhost:50051", grpc::InsecureChannelCredentials()));

    // Spawn reader thread that loops indefinitely
    std::thread thread_ = std::thread(&GreeterClient::AsyncCompleteRpc, &greeter);

    std::string user("world");
    greeter.SayHello(user);  // The actual RPC call!

    //std::cout << "Press control-c to quit" << std::endl << std::endl;
    thread_.join();

    return 0;
}
