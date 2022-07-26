/** 
 * @file client1b.cpp
 * @brief
 * Example of a client application.
 * This client application uses 1 CommClient object.
 *
 * This application is a variant of client1a.cpp.
 * The main difference between client1b.cpp and client1a.cpp
 * is that client1b.cpp applies co_await directly on the asynchronous operation or coroutine
 * without using intermediate async_operation or async_task objects.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <string>

#include <corolib/print.h>
#include <corolib/async_operation.h>
#include <corolib/commclient.h>
#include <corolib/async_task.h>

#include "endpoints.h"

using namespace corolib;

boost::asio::io_context ioContext;

class ClientApp : public CommClient
{
public:
    ClientApp(
        boost::asio::io_context & ioContext,
        boost::asio::ip::tcp::endpoint ep) :
        CommClient(ioContext, ep)
    {
        print(PRI1, "ClientApp::ClientApp(...)\n");
    }

    /**
     * @brief
     * mainflow opens a connection to a server object, 
     * prepares a string and writes this string to the object,
     * reads the response and closes the connection. 
     * @param
     * @param
     * @return
     */
    async_task<int> mainflow(int i, int& counter)
    {
        // Connecting
        print(PRI1, "mainflow: co_await start_connecting();\n");
        co_await start_connecting();

        std::string str1 = "This is string ";
        str1 += std::to_string(counter++);
        str1 += " to echo\n";

        // Writing
        print(PRI1, "mainflow: async_operation<void> co_await start_writing(...);\n");
        co_await start_writing(str1.c_str(), str1.length() + 1);

        // Reading
        print(PRI1, "mainflow: std::string strout = co_await start_reading();\n");
        std::string strout = co_await start_reading();
        print(PRI1, "mainflow: strout = %s", strout.c_str());

        // Just start a timer to introduce a delay to simulate a long asynchronous calculation 
        // after having read the response.
        // Delaying
        steady_timer client_timer(ioContext);
        print(PRI1, "mainflow: co_await start_timer(100);\n");
        co_await start_timer(client_timer, 100);

        // Closing
        print(PRI1, "mainflow: stop();\n");
        stop();

        co_return i;
    }

    /**
     * @brief
     * @param
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
                // Introduce a delay of 3 seconds to allow multiple client1x applications to be started and run in parallel.
                steady_timer client_timer(ioContext);
                print(PRI1, "mainflow: co_await start_timer(3000);\n");
                co_await start_timer(client_timer, 3000);
            }

            print(PRI1, "mainflow: int ret = co_await mainflow(i, counter);\n");
            int ret = co_await mainflow(i, counter);
            print(PRI1, "mainflow: ret = %d\n", ret);
        }

        print(PRI1, "mainflow: co_return 0;\n");
        co_return 0;
    }
};

int main()
{
    set_priority(0x01);

    print(PRI1, "main: ClientApp c1(ioContext, ep1);\n");
    ClientApp c1(ioContext, ep1);

    print(PRI1, "main: async_task<int> si = c1.mainflow();\n");
    async_task<int> si = c1.mainflow();

    print(PRI1, "main: before ioContext.run();\n");
    ioContext.run();
    print(PRI1, "main: after ioContext.run();\n");

    print(PRI1, "main: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    print(PRI1, "main: return 0;\n");
    return 0;
}
