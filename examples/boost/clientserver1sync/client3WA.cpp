/**
 * @file client3WA.cpp
 * @brief
 * Example of a client application.
 * Simplified version of ../clientserver1/client3WA.cpp.
 * 
 * @author Johan Vanslembrouck
 */

#include <string>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/when_all.h>

#include <commclientsync.h>

#include "endpoints.h"

using namespace corolib;

CommQueue myeventqueue;

async_task<int> mainflowOneClient(boost::asio::io_context &ioContext, CommClient& cl)
{
    print(PRI1, "mainflowOneClient: begin\n");

    for (int i = 0; i < 10; i++)
    {
        print(PRI1, "mainflowOneClient: %d ------------------------------------------------------------------\n", i);
 
        // Connecting
        print(PRI1, "mainflowOneClient: async_operation<void> sc = cl.start_connecting();\n");
        async_operation<void> sc = cl.start_connecting();
        print(PRI1, "mainflowOneClient: co_await sc;\n");
        co_await sc;

        std::string str1 = "This is string " + std::to_string(i) + " to echo\n";
        print(PRI1, "mainflowOneClient: sending to server: %s", str1.c_str());
        
        // Writing
        print(PRI1, "mainflowOneClient: async_operation<void> sw = cl.start_writing(...);\n");
        async_operation<void> sw = cl.start_writing(str1);
        print(PRI1, "mainflowOneClient: co_await sw;\n");
        co_await sw;

        // Reading
        print(PRI1, "mainflowOneClient: async_operation<std::string> sr = cl.start_reading();\n");
        async_operation <std::string> sr = cl.start_reading();
        print(PRI1, "mainflowOneClient: std::string strout = co_await sr;\n");
        std::string strout = co_await sr;
        print(PRI1, "mainflowOneClient: strout = %s", strout.c_str());

        print(PRI1, "mainflowOneClient: async_operation<void> st = cl.start_timer(100);\n");
        async_operation<void> st = cl.start_timer(100);
        print(PRI1, "mainflowOneClient: co_await st;\n");
        co_await st;

        print(PRI1, "mainflowOneClient: close\n");
        cl.close();
    }

    print(PRI1, "mainflowOneClient: end\n");

    co_return 0;
}

async_task<int> mainflow1(boost::asio::io_context& ioContext, CommClient& cl1)
{
    print(PRI1, "mainflow1: async_task<int> t1 = mainflowOneClient(ioContext, cl1);\n");
    async_task<int> t1 = mainflowOneClient(ioContext, cl1);
    print(PRI1, "mainflow1: co_await t1;\n");
    co_await t1;

    print(PRI1, "mainflow1: co_return 0;\n");
    co_return 0;
}

async_task<int> mainflow3(boost::asio::io_context& ioContext, CommClient& cl1, CommClient& cl2, CommClient& cl3)
{
    async_task<int> t1 = mainflowOneClient(ioContext, cl1);
    async_task<int> t2 = mainflowOneClient(ioContext, cl2);
    async_task<int> t3 = mainflowOneClient(ioContext, cl3);

    print(PRI1, "mainflow3: when_all wac(t1, t2, t3);\n");
    when_all wac(t1, t2, t3);
    print(PRI1, "mainflow3: co_await wac;\n");
    co_await wac;

    print(PRI1, "mainflow3: co_return 0;\n");
    co_return 0;
}

async_task<int> mainflow(boost::asio::io_context& ioContext, CommClient& cl1, CommClient& cl2, CommClient& cl3)
{
    print(PRI1, "mainflow: async_task<int> t1 = mainflow1(ioContext, cl1);\n");
    async_task<int> t1 = mainflow1(ioContext, cl1);
    print(PRI1, "mainflow: co_await t1;\n");
    co_await t1;

    print(PRI1, "mainflow: async_task<int> t3 = mainflow3(ioContext, cl1, cl2, cl3);\n");
    async_task<int> t3 = mainflow3(ioContext, cl1, cl2, cl3);
    print(PRI1, "mainflow: co_await t3;\n");
    co_await t3;

    print(PRI1, "mainflow: async_operation<void> st = start_timer(100, true);\n");
    async_operation<void> st = cl1.start_timer(100, true);
    print(PRI1, "mainflow: co_await st;\n");
    co_await st;

    co_return 0;
}

int main()
{
    set_priority(0x01);         // Use 0x03 to follow the flow in corolib

    boost::asio::io_context ioContext;

    print(PRI1, "main: CommClient c1(ioContext, ep1);\n");
    CommClient c1(ioContext, myeventqueue, ep1);

    print(PRI1, "main: CommClient c1(ioContext, ep1);\n");
    CommClient c2(ioContext, myeventqueue, ep1);

    print(PRI1, "main: CommClient c1(ioContext, ep1);\n");
    CommClient c3(ioContext, myeventqueue, ep1);

    print(PRI1, "main: task<int> si = c1.mainflow();\n");
    async_task<int> si = mainflow(ioContext, c1, c2, c3);

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
