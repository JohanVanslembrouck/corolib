/** 
 * @file client0.cpp
 * @brief
 * Using Boost ASIO to communicate between a client and an echo server.
 *  
 * This example calls the synchronous API functions connect, write, read and close in a function fileaccess().
 * Function fileaccess() is placed on a thread in function start_fileaccess().
 * start_fileaccess() is called in a loop in a coroutine mainflow().
 *
 * @author Johan Vanslembrouck
 */

#include <thread>
#include <iostream>
#include <string>
#include <functional>

#include <boost/asio.hpp>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/when_all.h>
#include <corolib/commservice.h>
#include <corolib/eventqueue.h>
#include <corolib/threadpool.h>

using namespace corolib;

#include "endpoints.h"

// ---------------------------------------------------------

constexpr int ARRAYSIZE = 16;   // Use 2^N

using FunctionVoidVoid = std::function<void(void)>;
using EventQueueFunctionVoidVoid = QueueThreadSafe<FunctionVoidVoid, ARRAYSIZE>;
using CommQueue = EventQueueFunctionVoidVoid;

CommQueue myeventqueue;

// ---------------------------------------------------------

class CommClient : public CommService
{
public:
    explicit CommClient(boost::asio::io_context& io_context,
        EventQueueFunctionVoidVoid& eventqueue,
        boost::asio::ip::tcp::endpoint ep)
        : m_socket(io_context)
        , m_eventqueue(eventqueue)
        , m_ep(ep)
    {
        print(PRI2, "%p: CommClient::CommClient()\n", this);
    }

    async_operation<std::string> start_fileaccess(int num);
    async_operation<void> start_dummy();

protected:
    std::string fileaccess(int num);

protected:
    ThreadPool m_pool{ 4 };

    boost::asio::ip::tcp::socket m_socket;
    CommQueue& m_eventqueue;
    boost::asio::ip::tcp::endpoint m_ep;
};

std::size_t completionCondition(
    std::string& buffer,
    const boost::system::error_code& /*error*/, /// let's ignore
    std::size_t bytes_transferred)
{
    //print(PRI1, "completionCondition: bytes_transferred = %ld\n", bytes_transferred);
    if (!bytes_transferred)
    {
        return 1;
    }
    return buffer[bytes_transferred - 1] == '\n' ? 0 : 1;
}

async_operation<std::string> CommClient::start_fileaccess(int instance)
{
    int index = get_free_index_ts();
    async_operation < std::string > ret{ this, index, true };
    print(PRI1, "CommClient::start_fileaccess(%d)\n", instance);
    m_pool.enqueue(
        [this, index, instance]
        {
            std::string answer = fileaccess(instance);
            m_eventqueue.push([this, index, answer] { completionHandler(index, answer); });
        }
    );
    return ret;
}

std::string CommClient::fileaccess(int id)
{
    print(PRI1, "fileaccess: begin\n");

    print(PRI1, "fileaccess: connect to server\n");
    m_socket.connect(m_ep);
    print(PRI1, "fileaccess: client %d got connected\n", id);

    const std::string message = "This is string " + std::to_string(id) + " to echo\n";
    print(PRI1, "fileaccess: sending to server: %s", message.c_str());
    print(PRI1, "fileaccess: write\n");
    boost::asio::write(m_socket, boost::asio::buffer(message));

    print(PRI1, "fileaccess: read\n");
    std::string answer;
    boost::asio::read(
        m_socket,
        boost::asio::dynamic_buffer(answer),
        std::bind(completionCondition, std::ref(answer), std::placeholders::_1, std::placeholders::_2));

    print(PRI1, "fileaccess: server replied: %s", answer.c_str());
    print(PRI1, "fileaccess: client %d got served\n", id);

    print(PRI1, "fileaccess: close\n");
    m_socket.close();

    print(PRI1, "fileaccess: end\n");

    return answer;
}

async_operation<void> CommClient::start_dummy()
{
    int index = get_free_index_ts();
    async_operation<void> ret{ this, index, true };
    m_pool.enqueue(
        [this, index]
        {
            m_eventqueue.pushFinal([this, index] { completionHandler_v(index); });
        }
    );
    return ret;
}

// ---------------------------------------------------------

class ClientApp : public CommClient
{
public:
    ClientApp(
        boost::asio::io_context& ioContext,
        CommQueue& eventqueue,
        boost::asio::ip::tcp::endpoint ep)
        : CommClient(ioContext, eventqueue, ep)
        , m_ioContext(ioContext)
    {
        print(PRI1, "ClientApp::ClientApp(...)\n");
    }

    async_task<int> mainflow()
    {
        print(PRI1, "ClientApp::mainflow: begin\n");

        for (int i = 0; i < 10; i++)
        {
            print(PRI1, "ClientApp::mainflow: %d ------------------------------------------------------------------\n", i);

            print(PRI1, "ClientApp::mainflow: async_operation<void> sfa = start_fileaccess(%d)\n", i);
            async_operation<std::string> sfa = start_fileaccess(i);

            print(PRI1, "ClientApp::mainflow: std::string answer = co_await sfa\n");
            std::string answer = co_await sfa;

            print(PRI1, "ClientApp::mainflow: answer = %s", answer.c_str());
        }

        print(PRI1, "ClientApp::mainflow: end\n");

        co_return 0;
    }

protected:
    boost::asio::io_context& m_ioContext;
};

async_task<int> mainflowOneClient(ClientApp& cl, int instance)
{
    print(PRI1, "mainflowOneClient %d: begin\n", instance);

    for (int i = 0; i < 10; i++)
    {
        print(PRI1, "mainflowOneClient %d: %d ------------------------------------------------------------------\n", instance, i);

        print(PRI1, "mainflowOneClient %d: async_operation<void> sfa = start_fileaccess(%d)\n", instance, i);
        async_operation<std::string> sfa = cl.start_fileaccess(i);
        print(PRI1, "mainflowOneClient %d: std::string answer = co_await sfa\n", instance);
        std::string answer = co_await sfa;

        print(PRI1, "mainflowOneClient %d: answer = %s", instance, answer.c_str());
    }

    print(PRI1, "mainflowOneClient %d: end\n", instance);

    co_return 0;
}

async_task<int> mainflow3(ClientApp& cl1, ClientApp& cl2, ClientApp& cl3)
{
    async_task<int> t1 = mainflowOneClient(cl1, 0);
    async_task<int> t2 = mainflowOneClient(cl2, 1);
    async_task<int> t3 = mainflowOneClient(cl3, 2);

    print(PRI1, "mainflow3: when_all wac(t1, t2, t3);\n");
    when_all wac(t1, t2, t3);
    print(PRI1, "mainflow3: co_await wac;\n");
    co_await wac;

    print(PRI1, "mainflow3: co_return 0;\n");
    co_return 0;
}

async_task<int> mainflow(ClientApp& cl1, ClientApp& cl2, ClientApp& cl3)
{
    print(PRI1, "main: async_task<int> t1 = cl1.mainflow();\n");
    async_task<int> t1 = cl1.mainflow();
    print(PRI1, "main: co_await t1;\n");
    co_await t1;

    print(PRI1, "main: async_task<int> t2 = mainflowOneClient(cl1, 0);\n");
    async_task<int> t2 = mainflowOneClient(cl1, 0);
    print(PRI1, "main: co_await t2;\n");
    co_await t2;

    print(PRI1, "mainflow: async_task<int> t3 = mainflow3(cl1, cl2, cl3);\n");
    async_task<int> t3 = mainflow3(cl1, cl2, cl3);
    print(PRI1, "mainflow: co_await t3;\n");
    co_await t3;

    print(PRI1, "mainflow: async_operation<void> sd = start_dummy();\n");
    async_operation<void> sd = cl1.start_dummy();
    co_await sd;
    print(PRI1, "mainflow: co_await sd;\n");

    co_return 0;
}

// ---------------------------------------------------------

// Use big size that supersedes the number of events in the applications
void runEventQueue(CommQueue& queue, int size = 100000)
{
    print(PRI5, "runEventQueue: begin\n");
    for (int i = 0; i < size; i++)
    {
        print(PRI5, "runEventQueue(): FunctionVoidVoid fun = queue.pop();\n");
        FunctionVoidVoid op = queue.pop();
        print(PRI5, "runEventQueue(): before op();\n");
        op();
        print(PRI5, "runEventQueue(): after op();\n");
        if (queue.stopped())
            break;
    }
    print(PRI5, "runEventQueue: end\n");
}

// ---------------------------------------------------------

int main()
{
    set_priority(0x01);         // Use 0x03 to follow the flow in corolib

    boost::asio::io_context ioContext;

    print(PRI1, "main: ClientApp c1(ioContext, myeventqueue, ep1);\n");
    ClientApp c1(ioContext, myeventqueue, ep1);

    print(PRI1, "main: ClientApp c1(ioContext, ep1);\n");
    ClientApp c2(ioContext, myeventqueue, ep1);

    print(PRI1, "main: ClientApp c1(ioContext, ep1);\n");
    ClientApp c3(ioContext, myeventqueue, ep1);

    print(PRI1, "main: task<int> si = c1.mainflow();\n");
    async_task<int> si = mainflow(c1, c2, c3);

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
