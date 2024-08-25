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

UseMode useMode;

int delay = 10;
int delay2 = 20;

ThreadAwaker *awaker = nullptr;

/**
 * @brief async_op
 * See explanation in use_mode.h.
 * 
 */
void async_op(std::function<void(int)>&& completionHandler)
{
    switch (useMode)
    {
    case UseMode::USE_NONE:
        // Nothing to be done here: eventHandler should be called "manually" by the application
        eventHandler = completionHandler;
        break;
    case UseMode::USE_EVENTQUEUE:
    {
        //std::function<void(void)> completionHandler1 = [completionHandler]() { completionHandler(10); };    // Not used yet
        eventQueue.push(std::move(completionHandler));
        break;
    }
    case UseMode::USE_THREAD:
    {
        if (awaker)
            awaker->addThread();

        std::thread thread1([completionHandler]() {
            print(PRI1, "async_op(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", delay);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));

            if (awaker)
                awaker->awaitRelease();

            print(PRI1, "async_op(): thread1: completionHandler(10);\n");
            completionHandler(10);
            print(PRI1, "async_op(): thread1: return;\n");
            });
        thread1.detach();

        // Normally we should not use delays in an asynchronous application.
        // The following delay is only used to investigate possible problems caused by it,
        // especially when the completionHandler in the thread runs before the following delay expires.
        //print(PRI1, "async_op(): before std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", delay2);
        //std::this_thread::sleep_for(std::chrono::milliseconds(delay2));
        //print(PRI1, "async_op(): after std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", delay2);

        break;
    }
    case UseMode::USE_THREAD_QUEUE:
    {
        queueSize++;

        std::thread thread1([completionHandler]() {
            print(PRI1, "async_op(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", delay);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));

            //std::function<void(void)> completionHandler2 = [completionHandler]() { completionHandler(10); };    // Not used yet
            std::function<void(int)> completionHandler1 = completionHandler;
            print(PRI1, "async_op(): thread1: eventQueueThr.push(std::move(completionHandler1));\n");
            eventQueueThr.push(std::move(completionHandler1));
            print(PRI1, "async_op(): thread1: return;\n");
            });
        thread1.detach();
        break;
    }
    case UseMode::USE_IMMEDIATE_COMPLETION:
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

#define CATCH_EXCEPTION_IN_COROUTINE5 1
#define THROW_EXCEPTION_FROM_COROUTINE5 0
#define CATCH_EXCEPTION_IN_COROUTINE4 1

async_task<int> coroutine5()
{
    print(PRI1, "coroutine5(): async_operation<int> op;\n");
    async_operation<int> op;
    print(PRI1, "coroutine5(): start_operation_impl(op);\n");
    start_operation_impl(op);
#if !CATCH_EXCEPTION_IN_COROUTINE5
    print(PRI1, "coroutine5(): before int v = co_await op;\n");
    int v = co_await op;
    print(PRI1, "coroutine5(): after int v = co_await op;\n");
#else
    int v = 0;
    try {
        print(PRI1, "coroutine5(): before v = co_await op;\n");
        v = co_await op;
        print(PRI1, "coroutine5(): after v = co_await op;\n");
    }
    catch (const std::system_error& ex) {
        print(PRI1, "coroutine5(): v = co_await op; raised system_error exception!\n");
    }
    catch (...) {
        print(PRI1, "coroutine5(): v = co_await op; raised ... exception!\n");
    }
#endif
#if THROW_EXCEPTION_FROM_COROUTINE5
    print(PRI1, "coroutine5(): throw std::system_error{ 234, std::system_category() }; \n");
    throw std::system_error{ 234, std::system_category() };
#endif
    print(PRI1, "coroutine5(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> coroutine4()
{
    print(PRI1, "coroutine4(): async_task<int> a = coroutine5();\n");
    async_task<int> a = coroutine5();
#if !CATCH_EXCEPTION_IN_COROUTINE4
    print(PRI1, "coroutine4(): int v = co_await a;\n");
    int v = co_await a;
#else
    int v = 0;
    try {
        print(PRI1, "coroutine4(): before v = co_await a;\n");
        v = co_await a;
        print(PRI1, "coroutine4(): after v = co_await a;\n");
    }
    catch (const std::system_error& ex) {
        print(PRI1, "coroutine4(): v = co_await a; raised system_error exception!\n");
    }
    catch (...) {
        print(PRI1, "coroutine4(): v = co_await a; raised ... exception!\n");
    }
#endif
    print(PRI1, "coroutine4(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> coroutine3()
{
    print(PRI1, "coroutine3(): async_task<int> a1 = coroutine4();\n");
    async_task<int> a1 = coroutine4();
    print(PRI1, "coroutine3(): int v1 = co_await a1;\n");
    int v1 = co_await a1;

    print(PRI1);
    print(PRI1, "coroutine3(): async_task<int> a2 = coroutine4();\n");
    async_task<int> a2 = coroutine4();
    print(PRI1, "coroutine3(): int v2 = co_await a2;\n");
    int v2 = co_await a2;

    print(PRI1);
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
