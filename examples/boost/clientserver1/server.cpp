/**
 * @file server.cpp
 * @brief
 * Example of a server application.
 * The application is client for the server application.
 * The application can also be server for client1, client1a, client1b, client1c, client3, client3WA and client3WAny,
 * if these clients connect to this application instead of to clientserver (see endpoints.h).
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */
 
#include <boost/asio/signal_set.hpp>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/commserver.h>
#include <corolib/async_operation.h>
#include <corolib/oneway_task.h>

#include "endpoints.h"

using namespace corolib;

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
     * @brief
     * mainflow_reading_writing reads the request received from the client, 
     * waits 500 ms to simulate a long calculation,
     * converts all characters in the received string to uppercase,
     * sends the response to the client
     * and closes the connection.
     * @param commClient
     * @return oneway_task
     */
    oneway_task mainflow_reading_writing(spCommCore commClient)
    {
        // Reading
        print(PRI1, "mainflow_reading_writing: async_operation<std::string> sr = start_reading(CommClient);\n");
        async_operation<std::string> sr = commClient->start_reading();
        print(PRI1, "mainflow_reading_writing: std::string strout = co_await sr;\n");
        std::string strout = co_await sr;
        print(PRI1, "mainflow_reading_writing: strout = %s\n", strout.c_str());

        // Just start a timer to introduce a delay between reading and writing
        // to simulate a long calculation.
        // Delaying
        boost::asio::steady_timer client_timer(m_IoContext);
        print(PRI1, "mainflow_reading_writing: async_operation<void> st = start_timer(client_timer, 500);\n");
        async_operation<void> st = commClient->start_timer(client_timer, 500);
        print(PRI1, "mainflow_reading_writing: co_await st;\n");
        co_await st;

        // Preparing output: convert the input string to uppercase
        std::string strtoecho = strout;
        for (auto& c : strtoecho) c = toupper(c);

        // Writing
        print(PRI1, "mainflow_reading_writing: async_operation<void> sw = start_writing(clientSession);\n");
        async_operation<void> sw = commClient->start_writing(strout.c_str(), strout.length() + 1);
        print(PRI1, "mainflow_reading_writing: co_await sw;\n");
        co_await sw;

        // Closing
        print(PRI1, "mainflow_reading_writing: clientSession->close();\n");
        commClient->stop();

        print(PRI1, "mainflow_reading_writing: co_return\n");
        co_return;
    }

    /**
     * @brief
     * mainflow implements a potentially infinite loop where it
     * accepts connections from clients.
     * For each new client connection, mainflow calls mainflow_reading_writing
     * to handle the request from that client.
     * mainflow does not (and can not) co_await the completion of mainflow_reading_writing.
     * Instead, mainflow immediately co_awaits new client connections.
     *
     * mainflow has the same implementation as mainflow in clientserver.cpp.
     *
     * @return always 0
     */
    async_task<int> mainflow()
    {
        int counter = 0;
        while (1)
        {
            print(PRI3, "mainflow: %d ------------------------------------------------------------------\n", counter++);
            spCommCore commCore = std::make_shared<CommCore>(m_IoContext);

            // Accepting
            print(PRI1, "mainflow: async_operation<void> sa = start_accepting(commCore);\n");
            async_operation<void> sa = start_accepting(commCore);
            print(PRI1, "mainflow: co_await sa;\n");
            co_await sa;

            // Start communicating asynchronously with the new client.
            // Start accepting new connections immediately.
            print(PRI1, "mainflow: mainflow_reading_writing(commCore);\n");
            (void)mainflow_reading_writing(commCore);
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
