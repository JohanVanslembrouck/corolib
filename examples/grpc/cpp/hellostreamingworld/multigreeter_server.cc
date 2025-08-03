/**
 * @file multigreeter_server.cc
 * @brief
 * Original source: https://groups.google.com/g/grpc-io/c/2wyoDZT5eao
 * @author: Kuldeep Melligeri
 */

#include <iostream>
#include <memory>
#include <string>

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>

#ifdef BAZEL_BUILD
#include "examples/protos/hellostreamingworld.grpc.pb.h"
#else
#include "hellostreamingworld.grpc.pb.h"
#endif

using grpc::Server;
using grpc::ServerAsyncResponseWriter;
using grpc::ServerBuilder;
using grpc::ServerCompletionQueue;
using grpc::ServerContext;
using grpc::Status;

using grpc::ServerAsyncWriter;

using hellostreamingworld::MultiGreeter;
using hellostreamingworld::HelloReply;
using hellostreamingworld::HelloRequest;

class ServerImpl final {
public:
    ~ServerImpl() {
        server_->Shutdown();
        // Always shutdown the completion queue after the server.
        cq_->Shutdown();
    }

    // There is no shutdown handling in this code.
    void Run() {
        std::string server_address("0.0.0.0:50051");

        ServerBuilder builder;
        // Listen on the given address without any authentication mechanism.
        builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
        // Register "service_" as the instance through which we'll communicate with
        // clients. In this case it corresponds to an *asynchronous* service.
        builder.RegisterService(&service_);
        // Get hold of the completion queue used for the asynchronous communication
        // with the gRPC runtime.
        cq_ = builder.AddCompletionQueue();
        // Finally assemble the server.
        server_ = builder.BuildAndStart();
        std::cout << "Server listening on " << server_address << std::endl;

        // Proceed to the server's main loop.
        HandleRpcs();
    }

private:
    // Class encompasing the state and logic needed to serve a request.
    class CallData {
    public:
        // Take in the "service" instance (in this case representing an asynchronous
        // server) and the completion queue "cq" used for asynchronous communication
        // with the gRPC runtime.
        CallData(MultiGreeter::AsyncService* service, ServerCompletionQueue* cq)
            : service_(service), cq_(cq), repliesSent_(0), responder_(&ctx_), status_(CREATE) {
            // Invoke the serving logic right away.
            Proceed();
        }

        void Proceed() {
            if (status_ == CREATE) {
                // Make this instance progress to the PROCESS state.
                status_ = PROCESS;
                std::cout << "Creating Call data for new client connections: " << this << std::endl;
                // As part of the initial CREATE state, we *request* that the system
                // start processing SayHello requests. In this request, "this" acts are
                // the tag uniquely identifying the request (so that different CallData
                // instances can serve different requests concurrently), in this case
                // the memory address of this CallData instance.
                service_->RequestsayHello(&ctx_, &request_, &responder_, cq_, cq_,
                                          (void*) this);
            } else if (status_ == PROCESS) {
                // Spawn a new CallData instance to serve new clients while we process
                // the one for this CallData. The instance will deallocate itself as
                // part of its FINISH state.
                new CallData(service_, cq_);

                // The actual processing.
                std::string prefix("Hello ");
                reply_.set_message(prefix + request_.name() +
                    std::to_string(repliesSent_ + 1));
                std::cout << "Sending reponse: " << this << " : " << reply_.message() << std::endl;
                responder_.Write(reply_, this);
                status_ = PROCESSING;
                repliesSent_++;

            } else if (status_ == PROCESSING) {
                if (repliesSent_ == MAX_REPLIES) {
                    // And we are done! Let the gRPC runtime know we've finished, using the
                    // memory address of this instance as the uniquely identifying tag for
                    // the event.
                    status_ = FINISH;
                    responder_.Finish(Status::OK, this);
                } else {
                    // The actual processing.
                    std::string prefix("Hello ");
                    reply_.set_message(prefix + request_.name() + std::to_string(repliesSent_ + 1));
                    std::cout << "Sending reponse: " << this << " : " << reply_.message() << std::endl;
                    responder_.Write(reply_, this);
                    status_ = PROCESSING;
                    repliesSent_++;
                }
            } else {
                GPR_ASSERT(status_ == FINISH);
                std::cout << "Completed RPC for: " << this << std::endl;
                // Once in the FINISH state, deallocate ourselves (CallData).
                delete this;
            }
        }

    private:
        // The means of communication with the gRPC runtime for an asynchronous
        // server.
        MultiGreeter::AsyncService* service_;
        // The producer-consumer queue where for asynchronous server notifications.
        ServerCompletionQueue* cq_;
        // Context for the rpc, allowing to tweak aspects of it such as the use
        // of compression, authentication, as well as to send metadata back to the
        // client.
        ServerContext ctx_;

        // What we get from the client.
        HelloRequest request_;
        // What we send back to the client.
        HelloReply reply_;

        uint32_t repliesSent_;
        const uint32_t MAX_REPLIES = 5;

        // The means to get back to the client.
        ServerAsyncWriter<HelloReply> responder_;

        // Let's implement a tiny state machine with the following states.
        enum CallStatus { CREATE, PROCESS, PROCESSING, FINISH };
        CallStatus status_;  // The current serving state.
    };

    // This can be run in multiple threads if needed.
    void HandleRpcs() {
        // Spawn a new CallData instance to serve new clients.
        new CallData(&service_, cq_.get());
        void* tag;  // uniquely identifies a request.
        bool ok;
        while (true) {
            // Block waiting to read the next event from the completion queue. The
            // event is uniquely identified by its tag, which in this case is the
            // memory address of a CallData instance.
            // The return value of Next should always be checked. This return value
            // tells us whether there is any kind of event or cq_ is shutting down.
            GPR_ASSERT(cq_->Next(&tag, &ok));
            GPR_ASSERT(ok);
            static_cast<CallData*>(tag)->Proceed();
        }
    }

    std::unique_ptr<ServerCompletionQueue> cq_;
    MultiGreeter::AsyncService service_;
    std::unique_ptr<Server> server_;
};

int main(int argc, char** argv) {
    ServerImpl server;
    server.Run();

    return 0;
}
