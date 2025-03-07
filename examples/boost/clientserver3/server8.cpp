/**
 * @file server8.cpp
 * @brief
 * This example illustrates the use of coroutines
 * in combination with Boost ASIO to implement a server application.
 * 
 * server8.cpp is a variant of server7.cpp.
 * server8.cpp uses "observer" couroutines, where one observer waits for and handles 1 specific request.
 * server8.cpp resumes the observer coroutine that is interested in that request.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
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

struct dispatch_table
{
    std::string     str;
    int             op;
};

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

    class Dispatcher
    {
    public:
        Dispatcher(ServerApp* serverapp)
            : m_serverapp(serverapp)
            , m_index(0)
            , m_running(true)
        {
            print(PRI1, "%p: Dispatcher::Dispatcher()\n", this);
            for (int i = 0; i < NROPERATIONS; i++)
            {
                m_dispatch_table[i].str = "EOF";
                m_dispatch_table[i].op = -1;
            }
        }

        void registerAsyncOperation(std::string tx, int index)
        {
            print(PRI1, "%p: Dispatcher::registerAsyncOperation(<%s>, %d): m_index = %d\n", this,  tx.c_str(), index, m_index);
            if (m_index < NROPERATIONS)
            {
                m_dispatch_table[m_index].str = tx;
                m_dispatch_table[m_index].op = index;
                m_index++;
            }
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

        bool dispatch(std::string str)
        {
            print(PRI2, "%p. Dispatcher::dispatch(str), m_index = %d, str = %s", this, m_index, str.c_str());

            std::string header = getHeader(str);

            for (int i = 0; i < m_index; i++)
            {
                print(PRI2, "%p: Dispatcher::dispatch(): m_dispatch_table[%d].str = <%s>\n", this,  i, m_dispatch_table[i].str.c_str());
                if (m_dispatch_table[i].str.compare(header) == 0)
                {
                    print(PRI1, "%p: Dispatcher::dispatch(...): found match at index %d => %d\n", this, i, m_dispatch_table[i].op);
                    
                    print(PRI1, "%p: Dispatcher::dispatch(...): before m_serverapp->callCompletionHandler(%d, %s:...)\n", this, m_dispatch_table[i].op, header.c_str());
                    m_serverapp->callCompletionHandler(m_dispatch_table[i].op, str);
                    print(PRI1, "%p: Dispatcher::dispatch(...): after m_serverapp->callCompletionHandler(%d, %s:...)\n", this, m_dispatch_table[i].op, header.c_str());
                    return true;
                }
            }
            return false;
        }

        void invokeAll(std::string str)
        {
            print(PRI1, "%p: Dispatcher::invokeAll(<%s>), m_index = %d\n", this, str.c_str(), m_index);
            for (int i = 0; i < m_index; i++)
            {
                m_serverapp->callCompletionHandler(m_dispatch_table[i].op, str);
            }
        }

    private:
        ServerApp* m_serverapp;
        dispatch_table m_dispatch_table[NROPERATIONS];
        int m_index;
    public:
        bool m_running;
    };

    void callCompletionHandler(int i, std::string str)
    {
        completionHandler<std::string>(i, str);
    }

    async_task<int> request1Observer(Dispatcher& dispatcher, ServerRequest& serverRequest)
    {
        print(PRI1, "request1Observer\n");
        static int counter = 0;

        int index = get_free_index();
        async_operation<std::string> op_str{ this, index };
        op_str.auto_reset(true);
        dispatcher.registerAsyncOperation("Req1", index);
        
        while (dispatcher.m_running)
        {
            print(PRI1, "request1Observer: std::string str = co_await op_str;\n");
            std::string str = co_await op_str;
            print(PRI1, "request1Observer: str = %s", str.c_str());
            if (dispatcher.m_running)
            {
                // TODO: unmarshal str into Req1
                Req1 req1;
                async_task<int> t = serverRequest.operation1(req1);
                //co_await t;       // Do not co_await!
            }
            counter++;
        }
        print(PRI1, "request1Observer: coreturn %d;\n", counter);
        co_return counter;
    }
    
    async_task<int> request2Observer(Dispatcher& dispatcher, ServerRequest& serverRequest)
    {
        print(PRI1, "request2Observer\n");
        static int counter = 0;

        int index = get_free_index();
        async_operation<std::string> op_str{ this, index };
        op_str.auto_reset(true);
        dispatcher.registerAsyncOperation("Req2", index);
        
        while (dispatcher.m_running)
        {
            print(PRI1, "request2Observer: std::string str = co_await op_str;\n");
            std::string str = co_await op_str;
            print(PRI1, "request2Observer: str = %s", str.c_str());
            if (dispatcher.m_running)
            {
                // TODO: unmarshal str into Req1
                Req2 req2;
                async_task<int> t = serverRequest.operation2(req2);
                //co_await t;       // Do not co_await!
            }
            counter++;
        }
        print(PRI1, "request2Observer: coreturn %d;\n", counter);
        co_return counter;
    }
    
    async_task<int> request3Observer(Dispatcher& dispatcher, ServerRequest& serverRequest)
    {
        print(PRI1, "request3Observer\n");
        static int counter = 0;

        int index = get_free_index();
        async_operation<std::string> op_str{ this, index };
        op_str.auto_reset(true);
        dispatcher.registerAsyncOperation("Req3", index);
        
        while (dispatcher.m_running)
        {
            print(PRI1, "request3Observer: std::string str = co_await op_str;\n");
            std::string str = co_await op_str;
            print(PRI1, "request3Observer: str = %s", str.c_str());
            if (dispatcher.m_running)
            {
                // TODO: unmarshal str into Req3
                Req3 req3;
                async_task<int> t = serverRequest.operation3(req3);
                //co_await t;       // Do not co_await!
            }
            counter++;
        }
        print(PRI1, "request3Observer: coreturn %d;\n", counter);
        co_return counter;
    }
    
    async_task<int> request4Observer(Dispatcher& dispatcher, ServerRequest& serverRequest)
    {
        print(PRI1, "request4Observer\n");
        static int counter = 0;

        int index = get_free_index();
        async_operation<std::string> op_str{ this, index };
        op_str.auto_reset(true);
        dispatcher.registerAsyncOperation("Req4", index);
        
        while (dispatcher.m_running)
        {
            print(PRI1, "request4Observer: std::string str = co_await op_str;\n");
            std::string str = co_await op_str;
            print(PRI1, "request4Observer: str = %s", str.c_str());
            if (dispatcher.m_running)
            {
                // TODO: unmarshal str into Req4
                Req4 req4;
                async_task<int> t = serverRequest.operation4(req4);
                //co_await t;       // Do not co_await!
            }
            counter++;
        }
        print(PRI1, "request4Observer: coreturn %d;\n", counter);
        co_return counter;
    }
    
    /**
     * @brief read_client_request uses a potentially infinite loop where
     * it reads the requests of one client.
     * It then dispatches the request to the operation registered by mainflow_one_client
     * and its co_awaits its completion.
     *
     * @param commClient is a shared pointer to a client object
     * @param dispatcher a reference to the dispatcher table populated by mainflow_one_client.
     * @return async_task<int> with value 0
     */
    async_task<int> read_client_request(spCommCore commClient, Dispatcher& dispatcher)
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
            print(PRI1, "read_client_request: std::string str = %s", str.c_str());

            if (str.compare("EOF") == 0)
            {
                dispatcher.m_running = false;
                break;
            }

            print(PRI1, "read_client_request: before dispatcher.dispatch(sr);\n");
            // In reality, str will contain the identification and the marshalled arguments
            if (!dispatcher.dispatch(str))
                print(PRI1, "read_client_request: dispatch failed!!!\n");
            print(PRI1, "read_client_request: after dispatcher.dispatch(sr);\n");
        }

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
     * @param commClient shared pointer to a client object
     * @return oneway_task
     */
    oneway_task main_one_client(spCommCore commClient)
    {
        print(PRI1, "main_one_client\n");

        Dispatcher dispatcher(this);
        ServerRequest serverRequest(commClient, m_IoContext);

        // Start the observer coroutines
        print(PRI1, "main_one_client: async_task<int> sr1 = request1Observer(dispatcher, serverRequest);\n");
        async_task<int> sr1 = request1Observer(dispatcher, serverRequest);
        print(PRI1, "main_one_client: async_task<int> sr2 = request2Observer(dispatcher, serverRequest);\n");
        async_task<int> sr2 = request2Observer(dispatcher, serverRequest);
        print(PRI1, "main_one_client: async_task<int> sr3 = request3Observer(dispatcher, serverRequest);\n");
        async_task<int> sr3 = request3Observer(dispatcher, serverRequest);
        print(PRI1, "main_one_client: async_task<int> sr4 = request4Observer(dispatcher, serverRequest);\n");
        async_task<int> sr4 = request4Observer(dispatcher, serverRequest);
        
        print(PRI1, "main_one_client: async_task<int> rcr = read_client_request(commClient, dispatcher);\n");
        async_task<int> rcr = read_client_request(commClient, dispatcher);
        print(PRI1, "main_one_client: int i = co_await rcr;\n");
        int i = co_await rcr;

        dispatcher.invokeAll("EOF");

        print(PRI1, "main_one_client: when_all obs({ &sr1, &sr2, &sr3, &sr4 });\n");
        when_all obs({ &sr1, &sr2, &sr3, &sr4 });
        print(PRI1, "main_one_client: co_await obs;\n");
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
