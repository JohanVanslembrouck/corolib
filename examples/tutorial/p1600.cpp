/**
 * @file p1600.cpp
 * @brief
 * Example with 9 coroutines.
 * 
 * coroutine1 co_awaits coroutine2 and coroutine12.
 * coroutineI (I = 2..4, 12..14) co_awaits coroutineI+1. 
 * coroutine3 calls coroutine4 twice and coroutine13 calls coroutine14 twice.
 * 
 * This example demonstrates "split-and-combine".
 * coroutine1 calls and co_awaits coroutine2 and coroutine12, which on their turn call other coroutines.
 * The coroutines at the deepest call level (coroutine5 and coroutine15) both co_await
 *    static async_operation<int> op;
 * coroutine15 starts the asynchronous operation and initializes the event handler.
 * The main function is responsible for calling the event handler.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <functional>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/when_all.h>

using namespace corolib;

std::function<void(int)> eventHandler;
static async_operation<int> op;

/**
 * @brief async_op
 *
 * @param completionHandler
 */
void async_op(std::function<void(int)>&& completionHandler)
{
    eventHandler = completionHandler;
}

/**
 * @brief start_operation_impl simulates starting an asynchronous operations.
 *
 * @param eventHandler is a reference to a std::function<void(int)> object.
 * @param op is a pointer to an async_operation<int> object.
 */
void start_operation_impl(std::function<void(int)>& eventHandler, async_operation<int>& op)
{
    print(PRI1, "start_operation_impl()\n");

    async_op([&op](int i)
        {
            print(PRI1, "completionHandler(): op->set_result(%d)\n", i);
            op.set_result(i);
            op.completed();
        });
}

async_task<int> coroutine15()
{
    print(PRI1, "coroutine15(): start_operation_impl(eventHandler, op);\n");
    start_operation_impl(eventHandler, op);

    print(PRI1, "coroutine15(): int v1 = co_await op;\n");
    int v = co_await op;

    print(PRI1, "coroutine15(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> coroutine14()
{
    print(PRI1, "coroutine14(): async_task<int> a = coroutine15();\n");
    async_task<int> a = coroutine15();
    print(PRI1, "coroutine14(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine14(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> coroutine13()
{
    print(PRI1, "coroutine13(): async_task<int> a1 = coroutine14();\n");
    async_task<int> a1 = coroutine14();
    print(PRI1, "coroutine13(): int v1 = co_await a1;\n");
    int v1 = co_await a1;

    print();
    print(PRI1, "coroutine13(): async_task<int> a2 = coroutine14();\n");
    async_task<int> a2 = coroutine14();
    print(PRI1, "coroutine13(): int v2 = co_await a2;\n");
    int v2 = co_await a2;

    print();
    print(PRI1, "coroutine13(): co_return v1+v2+1 = %d;\n", v1 + v2 + 1);
    co_return v1 + v2 + 1;
}

async_task<int> coroutine12()
{
    print(PRI1, "coroutine12(): async_task<int> a = coroutine3();\n");
    async_task<int> a = coroutine13();
    print(PRI1, "coroutine12(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine12(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> coroutine5()
{
    print(PRI1, "coroutine5(): int v1 = co_await op;\n");
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
    async_task<int> a1 = coroutine2();
    print(PRI1, "coroutine1(): async_task<int> a = coroutine12();\n");
    async_task<int> a2 = coroutine12();
    
    print(PRI1, "coroutine1(): when_all<async_operation<int>> wa({ &a1, &a2 });\n");
    when_all<async_task<int>> wa({ &a1, &a2 });
    
    print(PRI1, "coroutine1(): co_await wa;\n");
    co_await wa;

    print(PRI1, "coroutine1(): int v1 = a1.get_result();\n");
    int v1 = a1.get_result();
    print(PRI1, "coroutine1(): int v2 = a2.get_result();\n");
    int v2 = a2.get_result();
    
    print(PRI1, "coroutine1(): co_return v1+v2+1 = %d;\n", v1 + v2 + 1);
    co_return v1 + v2 + 1;
}
