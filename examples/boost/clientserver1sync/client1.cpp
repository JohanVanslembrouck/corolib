/**
 * @file client1.cpp
 * @brief
 * Example of a client application.
 * Simplified version of ../clientserver1/client1.cpp.
 * 
 * @author Johan Vanslembrouck
 */

#include <string>

#include <corolib/print.h>
#include <corolib/async_task.h>

#include <commclientsync.h>

#include "endpoints.h"

using namespace corolib;

CommQueue myeventqueue;

class ClientApp : public CommClient
{
public:
    ClientApp(
        boost::asio::io_context& ioContext,
        boost::asio::ip::tcp::endpoint ep)
        : CommClient(ioContext, myeventqueue, ep)
        , m_ioContext(ioContext)
    {
        print(PRI1, "ClientApp::ClientApp(...)\n");
    }

    async_task<int> mainflow()
    {
        print(PRI1, "mainflow: begin\n");

        for (int i = 0; i < 10; i++)
        {
            print(PRI1, "mainflow: %d ------------------------------------------------------------------\n", i);

            // Connecting
            print(PRI1, "mainflow: async_operation<void> sc = start_connecting();\n");
            async_operation<void> sc = start_connecting();
            print(PRI1, "mainflow: co_await sc;\n");
            co_await sc;

            std::string str1 = "This is string " + std::to_string(i) + " to echo\n";
            print(PRI1, "mainflow: sending to server: %s", str1.c_str());

            // Writing
            print(PRI1, "mainflow: async_operation<void> sw = start_writing(...);\n");
            async_operation<void> sw = start_writing(str1);
            print(PRI1, "mainflow: co_await sw;\n");
            co_await sw;

            // Reading
            print(PRI1, "mainflow: async_operation<std::string> sr = start_reading();\n");
            async_operation <std::string> sr = start_reading();
            print(PRI1, "mainflow: std::string strout = co_await sr;\n");
            std::string strout = co_await sr;
            print(PRI1, "mainflow: strout = %s", strout.c_str());

            print(PRI1, "mainflow: async_operation<void> st = start_timer(100);\n");
            async_operation<void> st = start_timer(100);
            print(PRI1, "mainflow: co_await st;\n");
            co_await st;

            print(PRI1, "mainflow: m_socket.close\n");
            m_socket.close();
        }

        print(PRI1, "mainflow: async_operation<void> st = start_timer(100, true);\n");
        async_operation<void> st = start_timer(100, true);
        co_await st;
        print(PRI1, "mainflow: co_await st;\n");

        print(PRI1, "mainflow: end\n");

        co_return 0;
    }

protected:
    boost::asio::io_context& m_ioContext;
};

int main()
{
    set_priority(0x01);         // Use 0x03 to follow the flow in corolib

    boost::asio::io_context ioContext;

    print(PRI1, "main: ClientApp c1(ioContext, ep1);\n");
    ClientApp c1(ioContext, ep1);

    print(PRI1, "main: task<int> si = c1.mainflow();\n");
    async_task<int> si = c1.mainflow();

    print(PRI1, "main: si.start();\n");
    si.start();

    print(PRI1, "main: before runEventQueue(myeventqueue);\n");
    runEventQueue(myeventqueue);
    print(PRI1, "main: after runEventQueue(myeventqueue);\n");

    print(PRI1, "main: si.get_result();\n");
    int res = si.get_result();
    print(PRI1, "main: res = %d;\n", res);

    print(PRI1, "main: return 0;\n");
    return 0;
}
