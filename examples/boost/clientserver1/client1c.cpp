/** 
 * @file client1c.cpp
 * @brief
 * Example of a client application.
 * This client application uses 1 CommClient object.
 *
 * This application is a variant of client1a.cpp.
 * Operations are placed in a local scope.
 * At regular points in mainflow(), the application does not co_await the completion of an operation.
 * Instead, it uses a timer to give the remote object some time to process the operation.
 * This is an experimental style. Do not use it for real applications!
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

// Use a number > 13 to avoid skipping co_await
const int MODULO_RESULT1 = 5;
const int MODULO_RESULT2 = 7;
int skipped_co_await = 0;

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
     * waits the response and closes the connection. 
     * Notice the use of the local scope to place async_operation objects in.
     * Experimental style!
     *
     */
    async_task<int> mainflow(int i, int& counter)
    {
        steady_timer client_timer(m_ioContext);

        if ((i % 11) == MODULO_RESULT1)
        {
            // Connecting
            print(PRI1, "mainflow: async_operation<void> sc = start_connecting();\n");
            async_operation<void> sc = start_connecting();
            print(PRI1, "mainflow: does not co_await start_connecting() !!!\n");
            skipped_co_await++;
            // We do not co_await sc here: use a timer instead to give the operation the time to complete.
            print(PRI1, "mainflow: co_await start_timer(100);\n");
            co_await start_timer(client_timer, 100);
        }
        else
        {
            print(PRI1, "mainflow: async_operation<void> sc = start_connecting();\n");
            async_operation<void> sc = start_connecting();
            print(PRI1, "mainflow: co_await sc;\n");
            co_await sc;
        }

        std::string str1 = "This is string ";
        str1 += std::to_string(counter++);
        str1 += " to echo\n";

        if ((i % 12) == MODULO_RESULT1)
        {
            // Writing
            print(PRI1, "mainflow: async_operation<void> sw = start_writing(...);\n");
            async_operation<void> sw = start_writing(str1.c_str(), str1.length() + 1);
            print(PRI1, "mainflow: does not co_await start_writing() !!!\n");
            skipped_co_await++;
            // We do not co_await sw here: use a timer instead to give the operation the time to complete.
            print(PRI1, "mainflow: co_await start_timer(100);\n");
            co_await start_timer(client_timer, 100);
        }
        else
        {
            print(PRI1, "mainflow: async_operation<void> sw = start_writing(...);\n");
            async_operation<void> sw = start_writing(str1.c_str(), str1.length() + 1);
            print(PRI1, "mainflow: co_await sw;\n");
            co_await sw;
        }

        if ((i % 13) == MODULO_RESULT1)
        {
            // Reading
            print(PRI1, "mainflow: async_operation<std::string> sr = start_reading();\n");
            async_operation<std::string> sr = start_reading();
            print(PRI1, "mainflow: does not co_await start_reading() !!!\n");
            skipped_co_await++;
            // We do not co_await sr here: use a timer instead to give the operation the time to complete.
            print(PRI1, "mainflow: co_await start_timer(100);\n");
            co_await start_timer(client_timer, 100);
        }
        else
        {
            print(PRI1, "mainflow: async_operation<std::string> sr = start_reading();\n");
            async_operation<std::string> sr = start_reading();
            print(PRI1, "mainflow: std::string strout = co_await sr;\n");
            std::string strout = co_await sr;
            print(PRI1, "mainflow: strout = %s", strout.c_str());
        }

        // Closing
        print(PRI1, "mainflow: stop();\n");
        stop(false);

        co_return i;
    }

    /**
     * @brief
     * @return always 0
     */
    async_task<int> mainflow1()
    {
        print(PRI1, "mainflow1: begin\n");
        int counter = 0;

        for (int i = 0; i < 100; i++)
        {
            print(PRI1, "mainflow1: %d ------------------------------------------------------------------\n", i);
            
            if (i == 0)
            {
                // Introduce a delay of 3 seconds to allow multiple client1x applications to be started and run in parallel.
                steady_timer client_timer(m_ioContext);
                print(PRI1, "mainflow1: co_await start_timer(3000);\n");
                co_await start_timer(client_timer, 3000);
            }

            if ((i % 14) == MODULO_RESULT2)
            {
                async_task<int> task = mainflow(i, counter);

                // We do not co_await task here: use a timer instead to give the task the time to complete.
                steady_timer client_timer(m_ioContext);
                print(PRI1, "mainflow1: co_await start_timer(1000);\n");
                co_await start_timer(client_timer, 1000);
            }
            else
            {
                print(PRI1, "mainflow1: int ret = co_await mainflow(i, counter);\n");
                int ret = co_await mainflow(i, counter);
                print(PRI1, "mainflow1: ret = %d\n", ret);
            }
        }

        print(PRI1, "mainflow1: co_return 0;\n");
        co_return 0;
    }

protected:
    boost::asio::io_context& m_ioContext;
};

int main()
{
    set_priority(0x01);

    boost::asio::io_context ioContext;

    print(PRI1, "main: ClientApp c1(ioContext, ep1);\n");
    ClientApp c1(ioContext, ep1);

    print(PRI1, "main: async_task<int> si = c1.mainflow1();\n");
    async_task<int> si = c1.mainflow1();

    print(PRI1, "main: before ioContext.run();\n");
    ioContext.run();
    print(PRI1, "main: after ioContext.run();\n");

    print(PRI1, "main: std::this_thread::sleep_for(std::chrono::seconds(1))\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    print(PRI1, "main: skipped_co_await = %d\n", skipped_co_await);

    print(PRI1, "main: return 0;\n");
    return 0;
}
