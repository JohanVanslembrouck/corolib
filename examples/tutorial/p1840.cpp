/**
 * @file p1840.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <functional>

#include "p1840.h"
#include "eventqueue.h"

using namespace corolib;

EventQueue eventQueue;
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
        // Nothing to be done here: completionHandler should be called "manually" by the application
        break;
    case USE_EVENTQUEUE:
        eventQueue.push(std::move(completionHandler));
        break;
    case USE_THREAD:
    {
        std::thread thread1([completionHandler]() {
            print(PRI1, "async_op: thread1: completionHandler(10);\n");
            completionHandler(10);
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
 * @brief start_operation_impl simulates starting an asynchronous operation
 * 
 * @param op is a reference to an async_operation<int> object.
 */
void start_operation_impl(async_operation<std::optional<int>>& op)
{
    print(PRI1, "start_operation_impl()\n");
    async_op(
        [&op](int i)
        {
            print(PRI1, "completionHandler(): op.set_result_and_complete(%d)\n", i);
            op.set_result_and_complete(i);
        });
}

async_operation<std::optional<int>> op4;
async_operation<std::optional<int>> op3;
async_operation<std::optional<int>> op2;
async_operation<std::optional<int>> op1;

async_task<int> coroutine3()
{
    print(PRI1, "coroutine3()\n");
    op3.auto_reset(true);
    std::optional<int> v1 = 0;
    int v = 0;
    
    do
    {
        print(PRI1, "coroutine3(): v1 = co_await op3;\n");
        v1 = co_await op3;
        print(PRI1, "coroutine3(): op4.set_result_and_complete(v1);\n");
        // Nothing ever co_awaits op4!
        op4.set_result_and_complete(v1);
        if (v1 != std::nullopt)
            v += *v1;
        else
            break;
    }
    while (true);
    
    print(PRI1, "coroutine3(): co_return v + 1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> coroutine2()
{
    print(PRI1, "coroutine2()\n");
    op2.auto_reset(true);
    std::optional<int> v1 = 0;
    int v = 0;

    do
    {
        print(PRI1, "coroutine2(): v1 = co_await op2;\n");
        v1 = co_await op2;
        print(PRI1, "coroutine2(): op3.set_result_and_complete(v1);\n");
        op3.set_result_and_complete(v1);
        if (v1 != std::nullopt)
            v += *v1;
        else
            break;
    }
    while (true);

    print(PRI1, "coroutine2(): co_return v + 1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> coroutine1()
{
    print(PRI1, "coroutine1()\n");
    op1.auto_reset(true);
    std::optional<int> v1 = 0;
    int v = 0;

    do
    {
        print(PRI1, "coroutine1(): v1 = co_await op1;\n");
        v1 = co_await op1;
        print(PRI1, "coroutine1(): op2.set_result_and_complete(v1);\n");
        op2.set_result_and_complete(v1);
        if (v1 != std::nullopt)
            v += *v1;
        else
            break;
    }
    while (true);
    
    print(PRI1, "coroutine1(): co_return v + 1 = %d;\n", v + 1);
    co_return v + 1;
}
