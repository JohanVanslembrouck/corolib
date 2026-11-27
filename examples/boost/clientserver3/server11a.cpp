/**
 * @file server11a.cpp
 * @brief
 * This example illustrates the use of coroutines
 * in combination with Boost ASIO to implement a server application.
 * 
 * server11a.cpp is an evolution of server10a.cpp.
 * 
 * This application uses the "chain of responsibility" design pattern,
 * where "observer" coroutines (request1Observer, request2Observer, request3Observer, request4Observer) 
 * decide if they handle a request or not.
 * 
 * server11a.cpp uses a simpler implementation than server10a.cpp: there is no need for a Dispatcher class.
 * It uses a async_operation_rmc<std::string> m_op_str data member, that is accessed in
 * request1Observer, request2Observer, request3Observer, request4Observer, 
 * callCompletionHandler and mainflow_one_client.
 * 
 * @author Johan Vanslembrouck
 */
 
#include <boost/asio/signal_set.hpp>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/oneway_task.h>
#include <corolib/when_all.h>

#include <commserver.h>

#include "endpoints.h"

using namespace corolib;

#include "serverrequest2.h"

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

    async_task<void> callCompletionHandler(std::string str)
    {
        m_op_str.set_result_and_complete(str);
        co_return;
    }

    void invokeAll(std::string str)
    {
        print(PRI1, "%p: Dispatcher::invokeAll: str = %s", this, str.c_str());
        callCompletionHandler(str);
    }

    std::string getHeader(std::string str)
    {
        while (str.size())
        {
            unsigned long index = str.find(':');
            if (index != std::string::npos)
            {
                return (str.substr(0, index));
            }
        }
        return "";
    }

    async_task<int> request1Observer(ServerRequest& serverRequest)
    {
        print(PRI1, "request1Observer\n");
        static int counter = 0;
        
        while (m_running)
        {
            print(PRI1, "request1Observer: std::string str = co_await op_str;\n");
            std::string str = co_await m_op_str;
            print(PRI1, "request1Observer: str = %s", str.c_str());
            if (m_running)
            {
                std::string header = getHeader(str);
                if (header.compare("Req1") == 0)
                {
                    m_handled = 1;
                    // TODO: unmarshal str into Req1
                    Req1 req1;
                    async_task<int> t = serverRequest.operation1(req1);
                    co_await t;
                }
            }
            counter++;
        }
        print(PRI1, "request1Observer: co_return %d;\n", counter);
        co_return counter;
    }
    
    async_task<int> request2Observer(ServerRequest& serverRequest)
    {
        print(PRI1, "request2Observer\n");
        static int counter = 0;

        while (m_running)
        {
            print(PRI1, "request2Observer: std::string str = co_await op_str;\n");
            std::string str = co_await m_op_str;
            print(PRI1, "request2Observer: str = %s", str.c_str());
            if (m_running)
            {
                std::string header = getHeader(str);
                if (header.compare("Req2") == 0)
                {
                    m_handled = 2;
                    // TODO: unmarshal str into Req1
                    Req2 req2;
                    async_task<int> t = serverRequest.operation2(req2);
                    co_await t;
                }
            }
            counter++;
        }
        print(PRI1, "request2Observer: co_return %d;\n", counter);
        co_return counter;
    }
    
    async_task<int> request3Observer(ServerRequest& serverRequest)
    {
        print(PRI1, "request3Observer\n");
        static int counter = 0;
        
        while (m_running)
        {
            print(PRI1, "request3Observer: std::string str = co_await op_str;\n");
            std::string str = co_await m_op_str;
            print(PRI1, "request3Observer: str = %s", str.c_str());
            if (m_running)
            {
                std::string header = getHeader(str);
                if (header.compare("Req3") == 0)
                {
                    m_handled = 3;
                    // TODO: unmarshal str into Req3
                    Req3 req3;
                    async_task<int> t = serverRequest.operation3(req3);
                    co_await t;
                }
            }
            counter++;
        }
        print(PRI1, "request3Observer: co_return %d;\n", counter);
        co_return counter;
    }
    
    async_task<int> request4Observer(ServerRequest& serverRequest)
    {
        print(PRI1, "request4Observer\n");
        static int counter = 0;

        while (m_running)
        {
            print(PRI1, "request4Observer: std::string str = co_await op_str;\n");
            std::string str = co_await m_op_str;
            print(PRI1, "request4Observer: str = %s", str.c_str());
            if (m_running)
            {
                std::string header = getHeader(str);
                if (header.compare("Req4") == 0)
                {
                    m_handled = 4;
                    // TODO: unmarshal str into Req4
                    Req4 req4;
                    async_task<int> t = serverRequest.operation4(req4);
                    co_await t;
                }
            }
            counter++;
        }
        print(PRI1, "request4Observer: co_return %d;\n", counter);
        co_return counter;
    }
    
    /**
     * @brief read_client_request uses a potentially infinite loop where
     * it reads the requests of one client.
     * It then dispatches the request to the operation registered by mainflow_one_client
     * and its co_awaits its completion.
     *
     * @param commClient is a shared pointer to a client object
     * @return async_task<int> with value 0
     */
    async_task<int> read_client_request(spCommCore commClient)
    {
        print(PRI1, "read_client_request: entry\n");

        while (1)
        {
            // Reading
            print(PRI1);
            print(PRI1, "read_client_request: async_operation<std::string> sr = commClient->start_reading();\n");
            async_operation<std::string> sr = commClient->start_reading();
            print(PRI1, "read_client_request: std::string str = co_await sr;\n");
            std::string str = co_await sr;
            print(PRI1, "read_client_request: str = %s", str.c_str());

            if (str.compare(0, 3, "EOF") == 0)
            {
                m_running = false;
                print(PRI1, "read_client_request: exiting loop\n");
                break;
            }

            // In reality, str will contain the identification and the marshalled arguments
            print(PRI1, "read_client_request: before co_await callCompletionHandler(str);\n");
            co_await callCompletionHandler(str);
            print(PRI1, "read_client_request: after co_await callCompletionHandler(str);\n");

            print(PRI1, "read_client_request: m_handled = %d\n", m_handled);
            if (m_handled == -1)
            {
                print(PRI1, "read_client_request: dispatch failed!!!\n");
                break;
            }
        }

        print(PRI1, "read_client_request: commClient->stop();\n");
        commClient->stop();

        print(PRI1, "read_client_request: co_return 0;\n");
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
     * @param commClient shared pointer to a client object
     * @return oneway_task
     */
    oneway_task mainflow_one_client(spCommCore commClient)
    {
        print(PRI1, "mainflow_one_client\n");

        m_running = true;
        m_handled = -1;

        ServerRequest serverRequest(commClient, m_IoContext);

        m_op_str.auto_reset(true);

        // Start the observer coroutines
        print(PRI1, "mainflow_one_client: async_task<int> sr1 = request1Observer(serverRequest);\n");
        async_task<int> sr1 = request1Observer(serverRequest);
        print(PRI1, "mainflow_one_client: async_task<int> sr2 = request2Observer(serverRequest);\n");
        async_task<int> sr2 = request2Observer(serverRequest);
        print(PRI1, "mainflow_one_client: async_task<int> sr3 = request3Observer(serverRequest);\n");
        async_task<int> sr3 = request3Observer(serverRequest);
        print(PRI1, "mainflow_one_client: async_task<int> sr4 = request4Observer(serverRequest);\n");
        async_task<int> sr4 = request4Observer(serverRequest);
        
        print(PRI1, "mainflow_one_client: async_task<int> rcr = read_client_request(commClient);\n");
        async_task<int> rcr = read_client_request(commClient);
        print(PRI1, "mainflow_one_client: int i = co_await rcr;\n");
        int i = co_await rcr;

        invokeAll("EOF\n");

        print(PRI1, "mainflow_one_client: when_all obs(sr1, sr2, sr3, sr4);\n");
        when_all obs(sr1, sr2, sr3, sr4);
        print(PRI1, "mainflow_one_client: co_await obs;\n");
        co_await obs;
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
            print(PRI1, "mainflow: mainflow_one_client(commCore);\n");
            (void)mainflow_one_client(commCore);
        }

        print(PRI1, "mainflow: co_return 0;\n");
        co_return 0;
    }

private:
    async_operation_rmc<std::string> m_op_str;
    bool m_running;
    int m_handled;
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
