/**
 * @file p1800.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <functional>

#include "p1800.h"
#include "eventqueue.h"

using namespace corolib;

EventQueue eventQueue;
std::function<void(int)> eventHandler;        // Will be initialized in start_operation_impl below

extern UseMode useMode;

/**
 * @brief start_operation_impl simulates starting an asynchronous operation and
 * initializing an event handler that will be called when that asynchronous operation completes.
 *
 * Starting the asynchronous operation is omitted in this implementation.
 * start_operation_impl initializes eventHandler with a lambda that will
 * be called on completion of the asynchronous operation.
 *
 * start_operation_impl is called from coroutine5 to start the asynchronous operation.
 *
 * In this example the main function will complete the operation by calling eventHandler.
 *
 * @param op is a pointer to an async_operation<int> object.
 */
void start_operation_impl(async_operation<int>* op)
{
    print(PRI1, "start_operation_impl()\n");

    // The asynchronous operation is normally started here, passing the eventHandler as one of its arguments.

    eventHandler = [op](int i)
    {
        print(PRI1, "eventHandler()\n");

        if (op)
        {
            print(PRI1, "eventHandler(): op->set_result(%d)\n", i);
            op->set_result(i);
            op->completed();
        }
        else
        {
            // This can occur when the async_operation_base has gone out of scope.
            print(PRI1, "eventHandler() : Warning: op == nullptr\n");
        }
    };

    switch (useMode)
    {
    case USE_NONE:
        // Nothing to be done here: eventHandler should be called "manually" by the application
        break;
    case USE_EVENTQUEUE:
        eventQueue.push(eventHandler);
        break;
    case USE_THREAD:
    {
        std::thread thread1([]() {
            print(PRI1, "Class01::start_operation_impl(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            print(PRI1, "Class01::start_operation_impl(): thread1: this->eventHandler(10);\n");
            eventHandler(10);
            print(PRI1, "Class01::start_operation_impl(): thread1: return;\n");
            });
        thread1.detach();
        break;
    }
    case USE_IMMEDIATE_COMPLETION:
        eventHandler(10);
        break;
    }
}

async_task<int> coroutine1()
{
    print(PRI1, "coroutine1(): async_operation<int> op;\n");
    async_operation<int> op;
	op.auto_reset(true);
	
    print(PRI1, "coroutine1(): start_operation_impl(&op);\n");
    start_operation_impl(&op);
    print(PRI1, "coroutine1(): int v1 = co_await op;\n");
    int v1 = co_await op;
		
    print(PRI1, "coroutine1(): start_operation_impl(&op);\n");
    start_operation_impl(&op);
    print(PRI1, "coroutine1(): int v2 = co_await op;\n");
    int v2 = co_await op;
		
    print(PRI1, "coroutine1(): start_operation_impl(&op);\n");
    start_operation_impl(&op);
    print(PRI1, "coroutine1(): int v3 = co_await op;\n");
    int v3 = co_await op;
		
    print(PRI1, "coroutine1(): start_operation_impl(&op);\n");
    start_operation_impl(&op);
    print(PRI1, "coroutine1(): int v4 = co_await op;\n");
    int v4 = co_await op;
	
    print(PRI1, "coroutine5(): co_return v1 + v2 + v3 + v4 + 1 = %d;\n", v1 + v2 + v3 + v4 + 1);
    co_return v1 + v2 + v3 + v4 + 1;
}
