/**
 * @file p1400.cpp
 * @brief
 * Example with 5 coroutines.
 * coroutineI (I = 1..4) co_awaits coroutineI+1.
 * coroutine3 calls coroutine4 twice.
 * coroutine5 starts an asynchronous operation and awaits its completion.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <functional>

#include "p1400.h"
#include "eventqueue.h"
#include "eventqueuethr.h"

using namespace corolib;

int queueSize = 0;

EventQueueFunctionVoidInt eventQueue;
EventQueueThrFunctionVoidInt eventQueueThr;

std::function<void(int)> eventHandler;        // Will be initialized in async_op

extern UseMode useMode;

/**
 * @brief async_op
 *
 */
void async_op(std::function<void(int)>&& completionHandler)
{
    switch (useMode)
    {
    case USE_NONE:
        // Nothing to be done here: eventHandler should be called "manually" by the application
        eventHandler = completionHandler;
        break;
    case USE_EVENTQUEUE:
        eventQueue.push(std::move(completionHandler));
        break;
    case USE_THREAD:
    {
        std::thread thread1([completionHandler]() {
            print(PRI1, "async_op: thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            print(PRI1, "async_op: thread1: completionHandler(10);\n");
            completionHandler(10);
            print(PRI1, "async_op: thread1: return;\n");
            });
        thread1.detach();
        break;
    }
    case USE_THREAD_QUEUE:
    {
        queueSize++;

        std::thread thread1([completionHandler]() {
            print(PRI1, "async_op: thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            std::function<void(int)> completionHandler1 = completionHandler;
            eventQueueThr.push(std::move(completionHandler1));
            print(PRI1, "async_op: thread1: return;\n");
            });
        thread1.detach();
        break;
    }
    case USE_IMMEDIATE_COMPLETION:
        completionHandler(10);
        break;
    }
}

/**
 * @brief start_operation_impl simulates starting an asynchronous operation;
 *
 * @param op is a pointer to an async_operation<int> object.
 */
void start_operation_impl(async_operation<int>& op)
{
    print(PRI1, "start_operation_impl()\n");
    async_op(
        [&op](int i)
        {
            print(PRI1, "completionHandler(): op->set_result(%d)\n", i);
            op.set_result_and_complete(i);
        });
}


async_task<int> coroutine5()
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

async_task<int> coroutine4()
{
    print(PRI1, "coroutine4(): async_task<int> a = coroutine5();\n");
    async_task<int> a = coroutine5();
    print(PRI1, "coroutine4(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine4(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> coroutine3()
{
    print(PRI1, "coroutine3(): async_task<int> a1 = coroutine4();\n");
    async_task<int> a1 = coroutine4();
    print(PRI1, "coroutine3(): int v1 = co_await a1;\n");
    int v1 = co_await a1;

    print();
    print(PRI1, "coroutine3(): async_task<int> a2 = coroutine4();\n");
    async_task<int> a2 = coroutine4();
    print(PRI1, "coroutine3(): int v2 = co_await a2;\n");
    int v2 = co_await a2;

    print();
    print(PRI1, "coroutine3(): co_return v1+v2+1 = %d;\n", v1 + v2 + 1);
    co_return v1 + v2 + 1;
}

async_task<int> coroutine2()
{
    print(PRI1, "coroutine2(): async_task<int> a = coroutine3();\n");
    async_task<int> a = coroutine3();
    print(PRI1, "coroutine2(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine2(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> coroutine1()
{
    print(PRI1, "coroutine1(): async_task<int> a = coroutine2();\n");
    async_task<int> a = coroutine2();
    print(PRI1, "coroutine1(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}
