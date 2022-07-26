/**
 * @file server.cpp
 * @brief
 * This example illustrates the use of coroutines
 * in combination with Boost ASIO to implement a server application.
 *
 * server.cpp is self-contained: it contains the complete implementation
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */
 
#include <boost/asio/signal_set.hpp>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/commserver.h>
#include <corolib/async_operation.h>
#include <corolib/oneway_task.h>
#include <corolib/wait_any_awaitable.h>

#include "endpoints.h"

using namespace corolib;

#include "reqresptypes.h"

using handleRequest = 
    std::function<void(std::string)>;

struct dispatch_table
{
    std::string     str;
    handleRequest    op;
};

class ServerApp : public CommServer
{
public:
    static const int NROPERATIONS = 128;
    
    /**
     * @brief ServerApp registers 4 operations together with their header string
     * in a dispatch table:
     * header string    operation
     * --------------------------
     * "Req1"           operation1
     * "Req2"           operation2
     * "Req3"           operation3
     * "Req4"           operation4
     */
    ServerApp(
        boost::asio::io_context& ioContext, unsigned short CommService) :
        CommServer(ioContext, CommService)
    {
        print(PRI1, "ServerApp::ServerApp(...)\n");
        m_index = 0;

        registerFunctor(
            "Req1",
            [this](std::string str)
            { 
                // TODO: unmarshal str into Req1
                Req1 req1;
                (void)operation1(req1); 
            });
                    
        registerFunctor(
            "Req2",
            [this](std::string str)
            {
                // TODO: unmarshal str into Req2
                Req2 req2;
                (void)operation2(req2); 
            });
                    
        registerFunctor(
            "Req3",
            [this](std::string str)
            {
                // TODO: unmarshal str into Req3
                Req3 req3;
                (void)operation3(req3); 
            });
                    
        registerFunctor(
            "Req4",
            [this](std::string str)
            {
                // TODO: unmarshal str into Req4
                Req4 req4;
                (void)operation4(req4);
            });
    }

    /**
     * @brief registerFunctor registers a handleRequest functor with a string in a lookup table.
     * registorFunctor does not check if the string already occurs in the table,
     * but simply adds it at the end of the table.
     * @param tx is the string to associate the handleRequest with
     * @param op is the handleRequest functor to associate with the string
     */
    void registerFunctor(std::string tx, handleRequest op)
    {
        print(PRI1, "ServerApp::registerFunctor(%s, op)\n", tx.c_str());
        if (m_index < NROPERATIONS)
        {
            m_dispatch_table[m_index].str = tx;
            m_dispatch_table[m_index].op = op;
            m_index++;
        }
    }
    
    /**
     * @brief operation1 writes a response to the client after a delay of 100 ms.
     * operation1 does not take the content of req1 into account and it prepares the response string
     * without marshalling it from a Resp1 object.
     * @param req1
     * @return oneway_task
     */
    oneway_task operation1(Req1 req1)
    {
        print(PRI1, "operation1(Req1 req1)\n");
        
        // Delaying
        steady_timer client_timer(m_IoContext);
        print(PRI1, "operation1: async_operation<void> st = m_commClient->start_timer(100);\n");
        async_operation<void> st = m_commClient->start_timer(client_timer, 100);
        print(PRI1, "operation1: co_await st;\n");
        co_await st;
            
        // Preparing output
        std::string strout = "Resp1:params-go-here\n";
        
        // Writing
        print(PRI1, "operation1: async_operation<void> sw = m_commClient->start_writing(...);\n");
        async_operation<void> sw = m_commClient->start_writing(strout.c_str(), strout.length() + 1);
        print(PRI1, "operation1: co_await sw;\n");
        co_await sw;
    }

    /**
     * @brief operation2 writes a response to the client after a delay of 200 ms.
     * operation2 does not take the content of req2 into account and it prepares the response string
     * without marshalling it from a Resp2 object.
     * @param req2
     * @return oneway_task
     */
    oneway_task operation2(Req2 req2)
    {
        print(PRI1, "operation2(Req2 req2)\n");
        
        // Delaying
        steady_timer client_timer(m_IoContext);
        print(PRI1, "operation2: async_operation<void> st = m_commClient->start_timer(200);\n");
        async_operation<void> st = m_commClient->start_timer(client_timer, 200);
        print(PRI1, "operation2: co_await st;\n");
        co_await st;
        
        // Preparing output
        std::string strout = "Resp2:params-go-here\n";

        // Writing
        print(PRI1, "operation2: async_operation<void> sw = m_commClient->start_writing(...);\n");
        async_operation<void> sw = m_commClient->start_writing(strout.c_str(), strout.length() + 1);
        print(PRI1, "operation2: co_await sw;\n");
        co_await sw;
    }

    /**
     * @brief operation3 writes a response to the client after a delay of 300 ms.
     * operation3 does not take the content of req3 into account and it prepares the response string
     * without marshalling it from a Resp3 object.
     * @param req3
     * @return oneway_task
     */
    oneway_task operation3(Req3 req3)
    {
        print(PRI1, "operation3(Req3 req3)\n");
        
        // Delaying
        steady_timer client_timer(m_IoContext);
        print(PRI1, "operation3: async_operation<void> st = m_commClient->start_timer(300);\n");
        async_operation<void> st = m_commClient->start_timer(client_timer, 300);
        print(PRI1, "operation3: co_await st;\n");
        
        // Preparing output
        std::string strout = "Resp3:params-go-here\n";

        // Writing
        print(PRI1, "operation3: async_operation<void> sw = m_commClient->start_writing(...);\n");
        async_operation<void> sw = m_commClient->start_writing(strout.c_str(), strout.length() + 1);
        print(PRI1, "operation3: co_await sw;\n");
        co_await sw;
    }

    /**
     * @brief operation4 writes a response to the client after a delay of 400 ms.
     * operation4 does not take the content of req4 into account and it prepares the response string
     * without marshalling it from a Resp4 object.
     * @param req4
     * @return oneway_task
     */
    oneway_task operation4(Req4 req4)
    {
        print(PRI4, "operation4(Req4 req4)\n");

        // Delaying
        steady_timer client_timer(m_IoContext);
        print(PRI1, "operation4: async_operation<void> st = m_commClient->start_timer(400);\n");
        async_operation<void> st = m_commClient->start_timer(client_timer, 400);
        print(PRI1, "operation4: co_await st;\n");
        
        // Preparing output
        std::string strout = "Resp4:params-go-here\n";

        // Writing
        print(PRI1, "operation4: async_operation<void> sw = m_commClient->start_writing(...);\n");
        async_operation<void> sw = m_commClient->start_writing(strout.c_str(), strout.length() + 1);
        print(PRI1, "operation4: co_await sw;\n");
        co_await sw;
    }
    
    /**
     * @brief getHeader retrieves the header identifying the operation from a string.
     * The header is the substring preceding the first : in the string.
     * @param str is the string to retrieve the header for,
     * @return the header found in the string or the empty string if not present
     */
    std::string getHeader(std::string str)
    {
        while (str.size())
        {
            int index = str.find(':');
            if (index != std::string::npos)
            {
                return str.substr(0, index);
            }
        }
        return "";
    }

    /**
     * @brief dispatch takes a string and calls (dispatches) the corresponding functor.
     * Function dispatch first retrieves the header from the string.
     * It then looks up the header in the dispatch table and calls the registered functor.
     * @param str is the input string
     */
    void dispatch(std::string str)
    {
        print(PRI2, "ServerApp::dispatch(<%s>), m_index = %d\n", str.c_str(), m_index);
        
        std::string header = getHeader(str);

        for (int i = 0; i < m_index; i++)
        {
            print(PRI2, "ServerApp::dispatch(): m_dispatch_table[%d].str = <%s>\n", i, m_dispatch_table[i].str.c_str());
            if (m_dispatch_table[i].str.compare(header) == 0)
            {
                print(PRI1, "ServerApp::dispatch(): found match at index %d\n", i);
                (void)m_dispatch_table[i].op(str);
                break;
            }
        }
    }
    
    /**
     * @brief mainflow_one_client handles the requests of one client.
     * mainflow_one_client uses a potentially infinite loop where
     * it reads the requests of one client.
     * It then dispatches the request to the operation registered by the constructor.
     * @param commClient shared pointer to a client object
     * @return oneway_task
     */
    oneway_task mainflow_one_client(spCommCore commClient)
    {
        print(PRI1, "mainflow_one_client: entry\n");
        m_commClient = commClient;
        
        while (1)
        {
            // Reading
            print(PRI1, "mainflow_one_client: async_operation<std::string> sr = commClient->start_reading();\n");
            async_operation<std::string> sr = commClient->start_reading();
            print(PRI1, "mainflow_one_client: std::string str = co_await sr;\n");
            std::string str = co_await sr;
            
            if (str.compare("EOF") == 0)
                break;

            print(PRI1, "mainflow_one_client: dispatcher.dispatch(sr);\n");
            // In reality, str will contain the identification and the marshalled arguments
            dispatch(str);
        }
        
        print(PRI1, "mainflow_one_client: commClient->stop();\n");
        commClient->stop();

        print(PRI1, "mainflow_one_client: co_return;\n");
        co_return;
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
    
protected:
    dispatch_table m_dispatch_table[NROPERATIONS];
    int m_index;
    spCommCore m_commClient;
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
