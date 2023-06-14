/** 
 * @file client5thr.cpp
 * @brief
 * Example of a client application.
 * This client application uses 3 CommClient objects that run in a thread of their own.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <string>

#include <corolib/print.h>
#include <corolib/async_operation.h>
#include <corolib/async_task.h>

#include <commclient.h>

#include "endpoints.h"

using namespace corolib;

class ClientApp : public CommClient
{
public:
    ClientApp(
        boost::asio::io_context & ioContext,
        boost::asio::ip::tcp::endpoint ep)
           : CommClient(ioContext, ep)
           , m_ioContext(ioContext)
    {
        print(PRI1, "ClientApp::ClientApp(...)\n");
    }

    /**
     * @brief
     * mainflow opens a connection to a server object, 
     * prepares a string and writes this string to the object,
     * reads the response and closes the connection. 
     * It repeats these actions 100 times.
     * @return always 0
     */
    async_task<int> mainflow()
    {
        print(PRI1, "mainflow: begin\n");
        int counter = 0;

        for (int i = 0; i < 100; i++)
        {
            print(PRI1, "mainflow: %d ------------------------------------------------------------------\n", i);

            if (i == 0)
            {
                // Introduce a delay of 3 seconds to allow multiple client1 applications to be started and run in parallel.
                steady_timer client_timer(m_ioContext);
                print(PRI1, "mainflow: co_await start_timer(3000);\n");
                co_await start_timer(client_timer, 3000);
            }

            // Connecting
            print(PRI1, "mainflow: async_operation<void> sc = start_connecting();\n");
            async_operation<void> sc = start_connecting();
            print(PRI1, "mainflow: co_await sc;\n");
            co_await sc;

            std::string str1 = "This is string ";
            str1 += std::to_string(counter++);
            str1 += " to echo\n";

            // Writing
            print(PRI1, "mainflow: async_operation<void> sw = start_writing(...);\n");
            async_operation<void> sw = start_writing(str1.c_str(), str1.length() + 1);
            print(PRI1, "mainflow: co_await sw;\n");
            co_await sw;

            // Reading
            print(PRI1, "mainflow: async_operation<std::string> sr = start_reading();\n");
            async_operation<std::string> sr = start_reading();
            print(PRI1, "mainflow: std::string strout = co_await sr;\n");
            std::string strout = co_await sr;
            print(PRI1, "mainflow: strout = %s", strout.c_str());

            // Just start a timer to introduce a delay to simulate a long asynchronous calculation 
            // after having read the response.
            // Delaying
            steady_timer client_timer(m_ioContext);
            print(PRI1, "mainflow: async_operation<void> st = start_timer(100);\n");
            async_operation<void> st = start_timer(client_timer, 100);
            print(PRI1, "mainflow: co_await st;\n");
            co_await st;

            // Closing
            print(PRI1, "mainflow: stop();\n");
            stop();
        }

        print(PRI1, "mainflow: co_return 0;\n");
        co_return 0;
    }

protected:
    boost::asio::io_context& m_ioContext;
};

int mainflow(int instance)
{
    boost::asio::io_context ioContext;

    print(PRI1, "main: ClientApp c1(ioContext, ep1);\n");
    ClientApp c1(ioContext, ep1);

    print(PRI1, "main: async_task<int> si = c1.mainflow();\n");
    async_task<int> si = c1.mainflow();

    print(PRI1, "main: before ioContext.run();\n");
    ioContext.run();
    print(PRI1, "main: after ioContext.run();\n");
    
    return instance;
}

int main()
{
    set_priority(0x01);

    print(PRI1, "main(): auto task1thr = std::async(std::launch::async, mainflow, 1);\n");
    auto task1thr = std::async(std::launch::async, mainflow, 1);
    print(PRI1, "main(): auto task2thr = std::async(std::launch::async, mainflow, 2);\n");
    auto task2thr = std::async(std::launch::async, mainflow, 2);
    print(PRI1, "main(): auto task3thr = std::async(std::launch::async, mainflow, 3);\n");
    auto task3thr = std::async(std::launch::async, mainflow, 3);
    
    print(PRI1, "main(): int v = task1thr.get() + task2thr.get() + task3thr.get();\n");
    int v = task1thr.get() + task2thr.get() + task3thr.get();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    print(PRI1, "main: return 0;\n");
    return 0;
}
