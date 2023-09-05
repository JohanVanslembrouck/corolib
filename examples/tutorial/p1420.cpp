/**
 * @file p1420.cpp
 * @brief
 * Example with 5 coroutines.
 * coroutineI (I = 1..4) co_awaits coroutineI+1.
 * coroutine3 calls coroutine4 twice.
 * coroutine5 starts an asynchronous operation on object01 and on object02 and awaits their completion.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/when_all.h>

using namespace corolib;

#include "class01.h"

extern Class01 object01;
extern Class01 object02;

#define CATCH_EXCEPTION_IN_COROUTINE5 1
#define THROW_EXCEPTION_FROM_COROUTINE5 0
#define CATCH_EXCEPTION_IN_COROUTINE4 1

async_task<int> coroutine5()
{
    print(PRI1, "coroutine5(): async_operation<int> op1 = object01.start_operation();\n");
    async_operation<int> op1 = object01.start_operation();
    print(PRI1, "coroutine5(): async_operation<int> op2 = object02.start_operation();\n");
    async_operation<int> op2 = object02.start_operation();
    print(PRI1, "coroutine5(): when_all wa({ &op1, &op2 });\n");
    when_all wa({ &op1, &op2 });
    int v = 0;
    print(PRI1, "coroutine5(): co_await wa;\n");
    co_await wa;
#if !CATCH_EXCEPTION_IN_COROUTINE5
    print(PRI1, "coroutine5(): int v = op1.get_result() + op2.get_result();\n");
    v = op1.get_result() + op2.get_result();
#else
    try {
       
        print(PRI1, "coroutine5(): int v = op1.get_result() + op2.get_result();\n");
        v = op1.get_result() + op2.get_result();
    }
    catch (const std::system_error& ex) {
        print(PRI1, "coroutine5(): v = op1.get_result() + op2.get_result(); raised system_error exception!\n");
    }
    catch (...) {
        print(PRI1, "coroutine5(): v = op1.get_result() + op2.get_result(); raised ... exception!\n");
    }
#endif
#if THROW_EXCEPTION_FROM_COROUTINE5
    print(PRI1, "coroutine5(): throw std::system_error{ 234, std::system_category() }; \n");
    throw std::system_error{ 234, std::system_category() };
#endif
    print(PRI1, "coroutine5(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> coroutine4()
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

async_task<int> coroutine3()
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

    print();
    print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}
