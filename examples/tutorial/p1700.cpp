/**
 * @file p1700.cpp
 * @brief
 * Example with 5 coroutines.
 * coroutineI (I = 1..4) co_awaits coroutineI+1.
 * coroutine3 calls coroutine4 twice.
 * coroutine5 starts an asynchronous operation and awaits its completion.
 * coroutine5 catches exceptions.
 * 
 * @author Johan Vanslembrouck
 */

#include "p1700.h"
#include "class00.h"

using namespace corolib;

async_ltask<int> coroutine5()
{
    print(PRI1, "coroutine5(): async_operation<int> op;\n");
    async_operation<int> op;
    print(PRI1, "coroutine5(): start_operation_impl(op);\n");
    start_operation_impl(op);

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
