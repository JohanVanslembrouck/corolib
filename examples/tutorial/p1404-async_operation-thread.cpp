/**
 * @file p1404-async_operation-thread.cpp
 * @brief
 * Starts an asynchronous operation that will be completed after one second from a thread.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <functional>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

// TODO: use local variable in start_operation_impl
std::function<void(int)> eventHandler;        // Will be initialized in start_operation_impl

/**
 * @brief start_operation_impl simulates starting an asynchronous operation and
 * initializing an event handler that will be called when that asynchronous operation completes.
 *
 * Starting the asynchronous operation is omitted in this implementation.
 * start_operation_impl initializes eventHandler with a lambda that will
 * be called on completion of the asynchronous operation.
 * start_operation_impl then starts a thread that will complete the operation after a delay of 1000 ms.
 *
 * start_operation_impl is called from coroutine5 in p1400.cpp.
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

    std::thread thread1([]() {
        print(PRI1, "start_operation_impl(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        print(PRI1, "start_operation_impl(): thread1: before operation(10);\n");
        eventHandler(10);
        print(PRI1, "start_operation_impl(): thread1: after operation(10);\n");
        });
    thread1.detach();
}

// Uses coroutine1 implemented in p1400.cpp
async_task<int> coroutine1();

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    print(PRI1, "main(): async_task<int> a = coroutine1();\n");
    async_task<int> a = coroutine1();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): return 0;\n");
    return 0;
}
