/**
 * @file server4.cpp
 * @brief
 * This example illustrates the use of coroutines
 * in combination with Boost ASIO to implement a server application.
 * 
 * This application is an extension of server3.cpp.
 * It uses a different Dispatcher implementation than server2.cpp and server3.cpp.
 * After having dispatched an operation (passed as a lambda), the dispatcher also completes
 * an asynchronous operation. This way main_one_client gets feedback on the invoked operation
 * and can (if applicable) take some additional actions.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */
 
#include <boost/asio/signal_set.hpp>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/oneway_task.h>
#include <corolib/when_any.h>

#include <commserver.h>

#include "endpoints.h"

using namespace corolib;

#include "dispatcher2.h"
#include "serverrequest.h"

class ServerApp : public CommServer
{
public:
    ServerApp(
        boost::asio::io_context& ioContext,
        unsigned short CommService) :
        CommServer(ioContext, CommService)
    {
        print(PRI1, "ServerApp::ServerApp(...)\n");
    }

    /**
     * @brief read_client_request uses a potentially infinite loop where
     * it reads the requests of one client.
     * It then dispatches the request to the operation registered by mainflow_one_client.
     * @param commClient is a shared pointer to a client object
     * @param dispatcher is a reference to the dispatcher table populated by mainflow_one_client.
     * @param done is a boolean passed by reference to indicate to its caller (mainflow_one_client)
     * that the client has finished processing.
     * @return async_task<int> with value 0
     */
    async_task<int> read_client_request(spCommCore commClient, Dispatcher& dispatcher, bool& done)
    {
        print(PRI1, "read_client_request: entry\n");
        while (1)
        {
            // Reading
            print(PRI1, "read_client_request: async_operation<std::string> sr = commClient->start_reading();\n");
            async_operation<std::string> sr = commClient->start_reading();
            print(PRI1, "read_client_request: std::string str = co_await sr;\n");
            std::string str = co_await sr;
            
            if (str.compare("EOF") == 0)
                break;

            print(PRI1, "read_client_request: dispatcher.dispatch(sr);\n");
            // In reality, str will contain the identification and the marshalled arguments
            dispatcher.dispatch(str);
        }

        done = true;
        print(PRI1, "mainflow_one_client: commClient->stop();\n");
        commClient->stop();

        print(PRI1, "mainflow_one_client: co_return 0;\n");
        co_return 0;
    }
    
    /**
     * @brief mainflow_one_client registers 4 operations together with their header string
     * in a dispatch table:
     * header string    operation
     * -----------------------------------------
     * "Req1"           serverRequest.operation1
     * "Req2"           serverRequest.operation2
     * "Req3"           serverRequest.operation3
     * "Req4"           serverRequest.operation4
     *
     * It then calls read_client_request that will handle the requests from that client.
     * It then enters a while loop where it co_awaits the completion of any of the
     * operations called from within read_client_request.
     * It will leave the loop when read_client_request indicates that the client
     * has finished.
     * @param commClient shared pointer to a client object
     * @return oneway_task
     */
    oneway_task main_one_client(spCommCore commClient)
    {
        Dispatcher dispatcher;
        ServerRequest serverRequest(commClient, m_IoContext);

        async_operation<std::string> reqs[4];

        reqs[0] = dispatcher.registerFunctor(
            "Req1",
            [&serverRequest](std::string str)
            {
                // TODO: unmarshal str into Req1
                Req1 req1;
                (void)serverRequest.operation1(req1);
            });

        reqs[1]  = dispatcher.registerFunctor(
            "Req2",
            [&serverRequest](std::string str)
            {
                // TODO: unmarshal str into Req2
                Req2 req2;
                (void)serverRequest.operation2(req2);
            });

        reqs[2] = dispatcher.registerFunctor(
            "Req3",
            [&serverRequest](std::string str)
            {
                // TODO: unmarshal str into Req3
                Req3 req3;
                (void)serverRequest.operation3(req3);
            });

        reqs[3] = dispatcher.registerFunctor(
            "Req4",
            [&serverRequest](std::string str)
            {
                // TODO: unmarshal str into Req4
                Req4 req4;
                (void)serverRequest.operation4(req4);
            });

        print(PRI1, "main_one_client: when_anyT<async_operation<std::string>> wat(reqs, 4);\n");
        when_anyT<async_operation<std::string>> wat(reqs, 4);

        bool done = false;
        print(PRI1, "main_one_client: async_task<int> rcr = read_client_request(commClient, dispatcher);\n");
        async_task<int> rcr = read_client_request(commClient, dispatcher, done);

        // Notice the while loops in read_client_request and here.
        while (!done)
        {
            print(PRI1, "main_one_client: int i = co_await wat;\n");
            int i = co_await wat;
            print(PRI1, "main_one_client: reqs[%d].get_result().c_str().get_result() = %s", i, reqs[i].get_result().c_str());
        }

        print(PRI1, "main_one_client: int i = co_await rcr;\n");
        int i = co_await rcr;
        (void)i;
    }

    /**
     * @brief mainflow uses a imfinite loop where it accepts connections
     * from clients. For every client it calls mainflow_one_client
     * that will handle the requests from that client.
     * It immediately starts accepting connections from new clients.
     * @return async_task<int> with value = 0
     */
    async_task<int> mainflow()
    {
        int counter = 0;
        while (1)
        {
            print(PRI1, "mainflow: %d ------------------------------------------------------------------\n", counter++);
            spCommCore commCore = std::make_shared<CommCore>(m_IoContext);

            // Accepting
            print(PRI1, "mainflow: async_operation<void> sa = start_accepting(commCore);\n");
            async_operation<void> sa = start_accepting(commCore);
            print(PRI1, "mainflow: co_await sa;\n");
            co_await sa;

            // Start communicating asynchronously with the new client.
            // Start accepting new connections immediately.
            print(PRI1, "mainflow: main_one_client(commCore);\n");
            (void)main_one_client(commCore);
        }

        print(PRI1, "mainflow: co_return 0;\n");
        co_return 0;
    }
};

std::atomic_bool stop{ false };

void runServer(
    ServerApp& server, 
    boost::asio::io_context& ioContext)
{
    print(PRI1, "runServer(...): async_task<int> si = server.mainflow();\n");
    async_task<int> si = server.mainflow();

    print(PRI1, "runServer(...): before ioContext.run();\n");
    ioContext.run();
    print(PRI1, "runServer(...): after ioContext.run();\n");
}

void handlerSignals(
    const boost::system::error_code& erc, 
    int signal)
{
    (void)erc;
    print(PRI1, "handlerSignals(...): signal %d occurred. Time to stop the application", signal);
    stop = true;
}

void asyncSignal(boost::asio::io_context& ioContext)
{
    print(PRI1, "asyncSignal(...)\n");
    boost::asio::signal_set signals{ioContext, SIGINT, SIGTERM};

    signals.async_wait(handlerSignals);
    ioContext.run();
}

int main()
{
    set_priority(0x01);

    boost::asio::io_context ioContextSignal;
    boost::asio::io_context ioContextServer;

    std::thread t{asyncSignal, std::ref(ioContextSignal)};

    ServerApp server1{ioContextServer, port_server};

    std::thread t1{runServer, std::ref(server1), std::ref(ioContextServer)};

    t.join(); /// wait on stop signal

    print(PRI1, "main(): server1.stop()\n");
    server1.stop();
    t1.join();

    print(PRI1, "main(): return 0;\n");
    return 0;
}
