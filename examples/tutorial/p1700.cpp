/**
 * @file p1700.cpp
 * @brief
 * Example with 5 coroutines.
 * coroutineI (I = 1..4) co_awaits coroutineI+1.
 * coroutine3 calls coroutine4 twice.
 * coroutine5 starts an asynchronous operation and awaits its completion.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <functional>

#include "p1700.h"
#include "eventqueue.h"
#include "eventqueuethr.h"

using namespace corolib;

int queueSize = 0;

EventQueueFunctionVoidInt eventQueue;
EventQueueThrFunctionVoidInt eventQueueThr;

std::function<void(int)> eventHandler;        // Will be initialized in start_operation_impl below

UseMode useMode;

/**
 * @brief async_op
 *
 */
void async_op(std::function<void(int)>&& completionHandler)
{
    switch (useMode)
    {
    case UseMode::USE_NONE:
        print(PRI1, "async_op(): UseMode::USE_NONE\n");
        // Nothing to be done here: eventHandler should be called "manually" by the application
        eventHandler = completionHandler;
        break;
    case UseMode::USE_EVENTQUEUE:
        print(PRI1, "async_op(): UseMode::USE_EVENTQUEUE\n");
        eventQueue.push(std::move(completionHandler));
        break;
    case UseMode::USE_THREAD:
    {
        print(PRI1, "async_op(): UseMode::USE_THREAD\n");
        std::thread thread1([completionHandler]() {
            print(PRI1, "async_op(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            print(PRI1, "async_op(): thread1: eventHandler(10);\n");
            completionHandler(10);
            print(PRI1, "async_op(): thread1: return;\n");
            });
        thread1.detach();
        break;
    }
    case UseMode::USE_THREAD_QUEUE:
    {
        print(PRI1, "async_op(): UseMode::USE_THREAD_QUEUE\n");
        queueSize++;

        std::thread thread1([completionHandler]() {
            print(PRI1, "async_op(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            std::function<void(int)> completionHandler1 = completionHandler;
            print(PRI1, "async_op(): thread1: eventQueueThr.push(std::move(completionHandler1));\n");
            eventQueueThr.push(std::move(completionHandler1));
            print(PRI1, "async_op(): thread1: return;\n");
            });
        thread1.detach();
        break;
    }
    case UseMode::USE_IMMEDIATE_COMPLETION:
        print(PRI1, "async_op(): UseMode::USE_IMMEDIATE_COMPLETION\n");
        completionHandler(10);
        break;
    }
}

/**
 * @brief start_operation_impl simulates starting an asynchronous operation.
 *
 * start_operation_impl is called from coroutine5 to start the asynchronous operation.
 *
 * In this example the main function will complete the operation by calling eventHandler.
 *
 * @param op is a pointer to an async_operation<int> object.
 */
void start_operation_impl(async_operation<int>& op)
{
    print(PRI1, "start_operation_impl()\n");

    async_op(
        [&op](int i)
        {
            if (i >= 0)
            {
                print(PRI1, "completionHandler(): op->set_result(%d)\n", i);
                op.set_result(i);
            }
            else
            {
                print(PRI1, "completionHandler(): op->set_error(%d)\n", i);
                op.set_error(i);
            }
            op.completed();
        });
}


async_ltask<int> coroutine5()
{
    print(PRI1, "coroutine5(): async_operation<int> op;\n");
    async_operation<int> op;
    print(PRI1, "coroutine5(): start_operation_impl(op);\n");
    start_operation_impl(op);
    print(PRI1, "coroutine5(): int v = co_await op;\n");
    int v = co_await op;
    print(PRI1, "coroutine5(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_ltask<int> coroutine4()
{
    print(PRI1, "coroutine4(): async_task<int> a5 = coroutine5();\n");
    async_ltask<int> a5 = coroutine5();
    print(PRI1, "coroutine4(): int v = co_await a5;\n");
    int v = co_await a5;
    print(PRI1, "coroutine4(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_ltask<int> coroutine3()
{
    print(PRI1, "coroutine3(): async_ltask<int> a41 = coroutine4();\n");
    async_ltask<int> a41 = coroutine4();
    print(PRI1, "coroutine3(): int v1 = co_await a41;\n");
    int v1 = co_await a41;

    print(PRI1);
    print(PRI1, "coroutine3(): async_ltask<int> a42 = coroutine4();\n");
    async_ltask<int> a42 = coroutine4();
    print(PRI1, "coroutine3(): int v2 = co_await a42;\n");
    int v2 = co_await a42;

    print(PRI1);
    print(PRI1, "coroutine3(): co_return v1+v2+1 = %d;\n", v1 + v2 + 1);
    co_return v1 + v2 + 1;
}

async_ltask<int> coroutine2()
{
    print(PRI1, "coroutine2(): async_ltask<int> a3 = coroutine3();\n");
    async_ltask<int> a3 = coroutine3();
    print(PRI1, "coroutine2(): int v = co_await a3;\n");
    int v = co_await a3;
    print(PRI1, "coroutine2(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_ltask<int> coroutine1()
{
    print(PRI1, "coroutine1(): async_ltask<int> a2 = coroutine2();\n");
    async_ltask<int> a2 = coroutine2();
    print(PRI1, "coroutine1(): int v = co_await a2;\n");
    int v = co_await a2;
    print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_ltask<void> coroutine0()
{
    print(PRI1, "coroutine0(): async_ltask<int> a1 = coroutine1();\n");
    async_ltask<int> a1 = coroutine1();
    print(PRI1, "coroutine0(): co_await a1;\n");
    co_await a1;
}

/**
 * @brief reinit has a void implementation for eager start coroutines.
 * It has been added to p1700.cpp because the main() function is the same
 * for eager start and lazy start coroutines and calls reinit().
 * 
 */
void reinit()
{
}
