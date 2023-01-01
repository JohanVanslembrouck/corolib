/**
 * @file p1500.cpp
 * @brief
 * Example with 7 coroutines.
 * coroutineI (I = 1..4, 11) co_awaits coroutineI+1.
 * coroutine3 calls coroutine4 twice.
 * coroutine5 and coroutine12 start an asynchronous operation and awaits its completion.
 *
 * This example is based on examples/clientserver3/server7.cpp where
 * a coroutine that is resumed from the event queue (coroutine5 in this case)
 * has to complete a coroutine that it has under its control (coroutine11 in this case).
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <functional>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

std::function<void(int)> eventHandler1;
std::function<void(int)> eventHandler2;

/**
 * @brief start_operation_impl simulates starting an asynchronous operation and
 * initializing an event handler that will be called when that asynchronous operation completes.
 *
 * Starting the asynchronous operation is omitted in this implementation.
 * start_operation_impl initializes eventHandler with a lambda that will
 * be called on completion of the asynchronous operation.
 *
 * @param eventHandler is a reference to a std::function<void(int)> object.
 * @param op is a pointer to an async_operation<int> object.
 */
void start_operation_impl(std::function<void(int)>& eventHandler, async_operation<int>& op)
{
    print(PRI1, "start_operation_impl()\n");

    // The asynchronous operation is normally started here, passing the eventHandler as one of its arguments.

    eventHandler = [&op](int i)
    {
        print(PRI1, "eventHandler(): op->set_result(%d)\n", i);
        op.set_result(i);
        op.completed();
    };
}

async_task<int> coroutine12()
{
    print(PRI1, "coroutine12(): async_operation<int> op;\n");
    async_operation<int> op;
    print(PRI1, "coroutine12(): start_operation_impl(&op);\n");
    start_operation_impl(eventHandler2, op);
    print(PRI1, "coroutine12(): int v = co_await op;\n");
    int v = co_await op;
    print(PRI1, "coroutine12(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> coroutine11()
{
    print(PRI1, "coroutine11(): async_task<int> a = coroutine12();\n");
    async_task<int> a = coroutine12();
    print(PRI1, "coroutine11(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine11(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> coroutine5()
{
    async_task<int> a = coroutine11();

    print(PRI1, "coroutine5(): async_operation<int> op;\n");
    async_operation<int> op;
    print(PRI1, "coroutine5(): start_operation_impl(&op);\n");
    start_operation_impl(eventHandler1, op);
    print(PRI1, "coroutine5(): int v1 = co_await op;\n");
    int v1 = co_await op;

    // Now complete coroutine12 and coroutine11 and then get the result
    print(PRI1, "coroutine5(): eventHandler2(20);\n");
    eventHandler2(20);
    print(PRI1, "coroutine5(): int v2 = a.get_result();\n");
    int v2 = a.get_result();

    print(PRI1, "coroutine5(): co_return v1+v2+1 = %d;\n", v1 + v2 + 1);
    co_return v1 + v2 + 1;
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
