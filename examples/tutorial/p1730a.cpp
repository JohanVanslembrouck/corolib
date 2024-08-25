/**
 * @file p1730a.cpp
 * @brief
 * Example with 6 coroutines.
 * coroutineI (I = 1..4) co_awaits coroutineI+1.
 * coroutine3 calls coroutine4 twice.
 * coroutine4 calls coroutine5a and coroutine5b and awaits the completion of both coroutines.
 * p1730a.cpp uses when_any instead of when_all (in p1730.cpp).
 * coroutine5a starts an asynchronous operation on object01 and awaits its completion.
 * coroutine5b starts an asynchronous operation on object02 and awaits its completion.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/when_any.h>

using namespace corolib;

#include "class01.h"

extern Class01 object01;
extern Class01 object02;

async_ltask<int> coroutine5a()
{
    print(PRI1, "coroutine5a(): async_operation<int> op = object01.start_operation();\n");
    async_operation<int> op = object01.start_operation();
    print(PRI1, "coroutine5a(): int v = co_await op;\n");
    int v = co_await op;
    print(PRI1, "coroutine5a(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_ltask<int> coroutine5b()
{
    print(PRI1, "coroutine5b(): async_operation<int> op = object02.start_operation();\n");
    async_operation<int> op = object02.start_operation();
    print(PRI1, "coroutine5b(): int v = co_await op;\n");
    int v = co_await op;
    print(PRI1, "coroutine5b(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_ltask<int> coroutine4()
{
    print(PRI1, "coroutine4(): async_ltask<int> a = coroutine5a();\n");
    async_ltask<int> a = coroutine5a();
    print(PRI1, "coroutine4(): async_ltask<int> b = coroutine5b();\n");
    async_ltask<int> b = coroutine5b();

    print(PRI1, "coroutine4(): when_any wa(a, b);\n");
    when_any wa(a, b);
	int idx = -1;
    for (int i = 0; i < 2; i++)
    {
        print(PRI1, "coroutine4(): idx = co_await wa;\n");
        idx = co_await wa;
        switch (idx)
        {
        case 0: print(PRI1, "coroutine4(): idx = %d: a.get_result() = %d\n", idx, a.get_result()); break;
        case 1: print(PRI1, "coroutine4(): idx = %d: b.get_result() = %d\n", idx, b.get_result()); break;
        default: print(PRI1, "coroutine4(): co_await wa returned invalid value %d\n", idx); break;
        }
    }
    print(PRI1, "coroutine4(): int v = a.get_result() + b.get_result();\n");
    int v = a.get_result() + b.get_result();
    print(PRI1, "coroutine4(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_ltask<int> coroutine3()
{
    print(PRI1, "coroutine3(): async_ltask<int> a1 = coroutine4();\n");
    async_ltask<int> a1 = coroutine4();
    print(PRI1, "coroutine3(): int v = co_await a1;\n");
    int v1 = co_await a1;

    print();
    print(PRI1, "coroutine3(): async_ltask<int> a2 = coroutine4();\n");
    async_ltask<int> a2 = coroutine4();
    print(PRI1, "coroutine3(): int v = co_await a2;\n");
    int v2 = co_await a2;

    print();
    print(PRI1, "coroutine3(): co_return v1+v2+1 = %d;\n", v1 + v2 + 1);
    co_return v1 + v2 + 1;
}

async_ltask<int> coroutine2()
{
    print(PRI1, "coroutine2(): async_ltask<int> a = coroutine3();\n");
    async_ltask<int> a = coroutine3();
    print(PRI1, "coroutine2(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine2(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_ltask<int> coroutine1() {
    print(PRI1, "coroutine1(): async_ltask<int> a = coroutine2();\n");
    async_ltask<int> a = coroutine2();
    print(PRI1, "coroutine1(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}
