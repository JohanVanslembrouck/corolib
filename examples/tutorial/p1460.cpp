/**
 * @file p1460.cpp
 * @brief
 * Example with 6 coroutines.
 * coroutineI (I = 1..4) co_awaits coroutineI+1.
 * coroutine3 calls coroutine4 twice.
 * coroutine4 calls coroutine5a and coroutine5b and awaits the completion of both coroutines.
 * coroutine5a starts twice two asynchronous operations on object01 and awaits their completion.
 * coroutine5b starts twice two asynchronous operations on object02 and awaits their completion.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/when_all.h>

using namespace corolib;

#include "p1460.h"

async_task<int> Class1460::coroutine5a()
{
    print(PRI1, "coroutine5a()\n");
    int v = 0;
    for (int i = 0; i < 2; i++)
    {
        print(PRI1, "coroutine5a(): async_operation<int> op1 = m_object01.start_operation1();\n");
        async_operation<int> op1 = m_object01.start_operation1();
        print(PRI1, "coroutine5a(): v += co_await op1;\n");
        v += co_await op1;

        print(PRI1, "coroutine5a(): async_operation<int> op2a = m_object01.start_operation2(1);\n");
        async_operation<int> op2a = m_object01.start_operation2(1);
        print(PRI1, "coroutine5a(): async_operation<int> op2a = m_object01.start_operation2(2);\n");
        async_operation<int> op2b = m_object01.start_operation2(2);
        print(PRI1, "when_all wa(op2a, op2b);\n");
        when_all wa(op2a, op2b);
        print(PRI1, "coroutine5a(): co_await wa;\n");
        co_await wa;
        print(PRI1, "coroutine5a(): int v = a.get() + b.get();\n");
        v += op2a.get_result() + op2b.get_result();
    }
    print(PRI1, "coroutine5a(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> Class1460::coroutine5b()
{
    print(PRI1, "coroutine5b()\n");
    int v = 0;
    for (int i = 0; i < 2; i++)
    {
        print(PRI1, "coroutine5b(): async_operation<int> op1 = m_object02.start_operation1();\n");
        async_operation<int> op1 = m_object02.start_operation1();
        print(PRI1, "coroutine5b(): v += co_await op1;\n");
        v += co_await op1;

        print(PRI1, "coroutine5b(): async_operation<int> op2a = m_object02.start_operation2(1);\n");
        async_operation<int> op2a = m_object02.start_operation2(1);
        print(PRI1, "coroutine5b(): async_operation<int> op2b = m_object02.start_operation2(2);\n");
        async_operation<int> op2b = m_object02.start_operation2(2);
        print(PRI1, "when_all wa(op2a, op2b);\n");
        when_all wa(op2a, op2b);
        print(PRI1, "coroutine5b(): co_await wa;\n");
        co_await wa;
        print(PRI1, "coroutine5b(): int v = a.get() + b.get();\n");
        v += op2a.get_result() + op2b.get_result();
    }
    print(PRI1, "coroutine5b(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> Class1460::coroutine4()
{
    print(PRI1, "coroutine4(): async_task<int> a = coroutine5a();\n");
    async_task<int> a = coroutine5a();
    print(PRI1, "coroutine4(): async_task<int> b = coroutine5b();\n");
    async_task<int> b = coroutine5b();
    print(PRI1, "coroutine4(): when_all wa(a, b);\n");
    when_all wa(a, b);
    print(PRI1, "coroutine4(): co_await wa;\n");
    co_await wa;
    print(PRI1, "coroutine4(): int v = a.get_result() + b.get_result();\n");
    int v = a.get_result() + b.get_result();
    print(PRI1, "coroutine4(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> Class1460::coroutine3()
{
    print(PRI1, "coroutine3(): async_task<int> a1 = coroutine4();\n");
    async_task<int> a1 = coroutine4();
    print(PRI1, "coroutine3(): int v = co_await a1;\n");
    int v1 = co_await a1;

    print(PRI1);
    print(PRI1, "coroutine3(): async_task<int> a2 = coroutine4();\n");
    async_task<int> a2 = coroutine4();
    print(PRI1, "coroutine3(): int v = co_await a2;\n");
    int v2 = co_await a2;

    print(PRI1);
    print(PRI1, "coroutine3(): co_return v1+v2+1 = %d;\n", v1 + v2 + 1);
    co_return v1 + v2 + 1;
}

async_task<int> Class1460::coroutine2()
{
    print(PRI1, "coroutine2(): async_task<int> a = coroutine3();\n");
    async_task<int> a = coroutine3();
    print(PRI1, "coroutine2(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine2(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> Class1460::coroutine1() {
    print(PRI1, "coroutine1(): async_task<int> a = coroutine2();\n");
    async_task<int> a = coroutine2();
    print(PRI1, "coroutine1(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}
