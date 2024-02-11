/**
 * @file p1410.cpp
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

using namespace corolib;

#include "p1410.h"

#define CATCH_EXCEPTION_IN_COROUTINE5 1
#define THROW_EXCEPTION_FROM_COROUTINE5 0
#define CATCH_EXCEPTION_IN_COROUTINE4 1

async_task<int> Class1410::coroutine5()
{
    print(PRI1, "coroutine5(): async_operation<int> op = object01.start_operation()\n");
    async_operation<int> op = m_object01.start_operation();
#if !CATCH_EXCEPTION_IN_COROUTINE5
    print(PRI1, "coroutine5(): before v = co_await op;\n");
    int v = co_await op;
    print(PRI1, "coroutine5(): after v = co_await op;\n");
#else
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
#endif
#if THROW_EXCEPTION_FROM_COROUTINE5
    print(PRI1, "coroutine5(): throw std::system_error{ 234, std::system_category() }; \n");
    throw std::system_error{ 234, std::system_category() };
#endif
    print(PRI1, "coroutine5(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> Class1410::coroutine4()
{
    print(PRI1, "coroutine4(): async_task<int> a = coroutine5();\n");
    async_task<int> a = coroutine5();
#if !CATCH_EXCEPTION_IN_COROUTINE4
    print(PRI1, "coroutine4(): int v = co_await a;\n");
    int v = co_await a;
#else
    int v = 0;
    try {
        print(PRI1, "coroutine4(): before v = co_await a;\n");
        v = co_await a;
        print(PRI1, "coroutine4(): after v = co_await a;\n");
    }
    catch (const std::system_error& ex) {
        print(PRI1, "coroutine4(): v = co_await a; raised system_error exception!\n");
    }
    catch (...) {
        print(PRI1, "coroutine4(): v = co_await a; raised ... exception!\n");
    }
#endif
    print(PRI1, "coroutine4(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> Class1410::coroutine3()
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

async_task<int> Class1410::coroutine2()
{
    print(PRI1, "coroutine2(): async_task<int> a = coroutine3();\n");
    async_task<int> a = coroutine3();
    print(PRI1, "coroutine2(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine2(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> Class1410::coroutine1()
{
    print(PRI1, "coroutine1(): async_task<int> a = coroutine2();\n");
    async_task<int> a = coroutine2();
    print(PRI1, "coroutine1(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}
