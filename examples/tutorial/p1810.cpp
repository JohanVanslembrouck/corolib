/**
 * @file p1810.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <functional>
#include <corolib/when_all.h>

#include "p1800.h"
#include "eventqueue.h"
#include "eventqueuethr.h"

using namespace corolib;

int queueSize = 0;

EventQueueFunctionVoidInt eventQueue;
EventQueueThrFunctionVoidInt eventQueueThr;

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
        // Nothing to be done here: completionHandler should be called "manually" by the application
        break;
    case UseMode::USE_EVENTQUEUE:
        eventQueue.push(std::move(completionHandler));
        break;
    case UseMode::USE_THREAD:
    {
        std::thread thread1([completionHandler]() {
            print(PRI1, "async_op(): thread1: completionHandler(10);\n");
            completionHandler(10);
            print(PRI1, "async_op(): thread1: return;\n");
            });
        thread1.detach();
        break;
    }
    case UseMode::USE_THREAD_QUEUE:
    {
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
        completionHandler(10);
        break;
    }
}

/**
 * @brief start_operation_impl simulates starting an asynchronous operation
 * 
 * @param op is a reference to an async_operation<int> object.
 */
void start_operation_impl(async_operation<int>& op)
{
    print(PRI1, "start_operation_impl()\n");
    async_op(
        [&op](int i)
        {
            print(PRI1, "completionHandler(): op.set_result_and_complete(%d)\n", i);
            op.set_result_and_complete(i);
        });
}

async_operation<int> op;
   
async_task<int> coroutine1a()
{
    print(PRI1, "coroutine1a(): async_operation<int> op;\n");
    int v1 = 0;
    int v = 0;
    
    do
    {
        print(PRI1, "coroutine1a(): v1 = co_await op;\n");
        v1 = co_await op;
        v += v1;
    }
    while (v1 != 0);
    
    print(PRI1, "coroutine1a(): co_return v + 1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> coroutine1b()
{
    print(PRI1, "coroutine1b(): async_operation<int> op;\n");
    int v1 = 0;
    int v = 0;
    
    do
    {
        print(PRI1, "coroutine1b(): int v1 = co_await op;\n");
        v1 = co_await op;
        v += v1;
    }
    while (v1 != 0);
    
    print(PRI1, "coroutine1b(): co_return v + 1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> coroutine1()
{
    print(PRI1, "coroutine1(): async_operation<int> op;\n");
    op.auto_reset(true);
   
    async_task<int> t1 = coroutine1a();
    async_task<int> t2 = coroutine1b();
#if 0
	// The following statement does not compile with g++ 11.3.0
    co_await when_all( { &t1, &t2 } );
#else
	// Split it into two lines:
    when_all wa({ &t1, &t2 });
	co_await wa;
#endif
    int v = t1.get_result() + t2.get_result();
    
    print(PRI1, "coroutine1(): co_return v + 1 = %d;\n", v + 1);
    co_return v + 1;
}
