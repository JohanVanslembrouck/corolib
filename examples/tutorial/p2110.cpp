/**
 * @file p2110.cpp
 * @brief
 * Example with 5 coroutines.
 * coroutineI (I = 1..4) co_awaits coroutineI+1.
 * coroutine3 calls coroutine4 twice.
 * coroutine5 starts an asynchronous operation on object01 and awaits its completion.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

#include "p2110.h"

using namespace corolib;


async_task<int> CoroClass01::coroutine5()
{
    print(PRI1, "coroutine5(): async_operation<int> op = m_object.start_operation()\n");
    async_operation<int> op = m_object.start_operation();
    print(PRI1, "coroutine5(): int v = co_await op;\n");
    int v = co_await op;
    print(PRI1, "coroutine5(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> CoroClass01::coroutine4()
{
    print(PRI1, "coroutine4(): async_task<int> a = coroutine5();\n");
    async_task<int> a = coroutine5();
    print(PRI1, "coroutine4(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine4(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> CoroClass01::coroutine3()
{
    print(PRI1, "coroutine3(): async_task<int> a1 = coroutine4();\n");
    async_task<int> a1 = coroutine4();
    print(PRI1, "coroutine3(): int v = co_await a1;\n");
    int v1 = co_await a1;

    print();
    print(PRI1, "coroutine3(): async_task<int> a2 = coroutine4();\n");
    async_task<int> a2 = coroutine4();
    print(PRI1, "coroutine3(): int v = co_await a2;\n");
    int v2 = co_await a2;

    print();
    print(PRI1, "coroutine3(): co_return v1+v2+1 = %d;\n", v1 + v2 + 1);
    co_return v1 + v2 + 1;
}

async_task<int> CoroClass01::coroutine2()
{
    print(PRI1, "coroutine2(): async_task<int> a = coroutine3();\n");
    async_task<int> a = coroutine3();
    print(PRI1, "coroutine2(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine2(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> CoroClass01::coroutine1()
{
    print(PRI1, "coroutine1(): async_task<int> a = coroutine2();\n");
    async_task<int> a = coroutine2();
    print(PRI1, "coroutine1(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

int task1(CoroClass01&& coroObject)
{
    print(PRI1, "task1: enter\n");
    async_task<int> res = [&coroObject]() -> async_task<int> {
        print(PRI1, "task1: async_task<int> a1 = coroObject.coroutine1();\n");
        async_task<int> a1 = coroObject.coroutine1();
        print(PRI1, "task1: int v1 = co_await a1;\n");
        int v1 = co_await a1;
        print(PRI1, "task1: co_return v1 = %d\n", v1);
        co_return v1;
    }();

    print(PRI1, "task1(): completionflow(coroObject);\n");
    completionflow(coroObject);

    print(PRI1, "task1: int v = res.get_result();\n");
    int v = res.get_result();
    print(PRI1, "task1: return v = %d;\n", v);
    return v;
}
