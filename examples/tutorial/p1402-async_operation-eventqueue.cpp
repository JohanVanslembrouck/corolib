/**
 * @file p1402-async_operation-eventqueue.cpp
 * @brief
 * Starts an asynchronous operation that will be completed from the main() function running the event queue.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <functional>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

#include "eventqueue.h"

using namespace corolib;

// TODO: use local variable in start_operation_impl
std::function<void(int)> eventHandler;        // Will be initialized in start_operation_impl

EventQueue eventQueue;

/**
 * @brief start_operation_impl simulates starting an asynchronous operation and
 * initializing an event handler that will be called when that asynchronous operation completes.
 *
 * Starting the asynchronous operation is omitted in this implementation.
 * start_operation_impl initializes eventHandler with a lambda that will
 * be called on completion of the asynchronous operation.
 * start_operation_impl then places the lambda in the eventQueue.
 *
 * start_operation_impl is called from coroutine5 in p1400.cpp to start the asynchronous operation.
 *
 * In this example the main function will complete the operation by running the event queue.
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
    
    eventQueue.push(eventHandler);
}

// Uses coroutine1 implemented in p1400.cpp
async_task<int> coroutine1();

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    print(PRI1, "main(): async_task<int> a = coroutine1();\n");
    async_task<int> a = coroutine1();

    print(PRI1, "main():  eventQueue.run();\n");
    eventQueue.run();

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): return 0;\n");
    return 0;
}
