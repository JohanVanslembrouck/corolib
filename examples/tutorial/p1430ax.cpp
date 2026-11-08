/**
 * @file p1430ax.cpp
 * @brief
 * Example with 6 coroutines.
 * coroutineI (I = 1..4) co_awaits coroutineI+1.
 * coroutine3 calls coroutine4 twice.
 * coroutine4 calls coroutine5a and coroutine5b and awaits the completion of both coroutines.
 * p1430a.cpp uses when_any instead of when_all (in p1430.cpp).
 * coroutine5a starts an asynchronous operation on object01 and awaits its completion.
 * coroutine5b starts an asynchronous operation on object02 and awaits its completion.
 *
 * @author Johan Vanslembrouck
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/when_any.h>

using namespace corolib;

#include "p1430a.h"

async_task<int> Class1430a::coroutine5a()
{
    print(PRI1, "coroutine5a(): async_operation<int> op = m_object01.start_operation();\n");
    async_operation<int> op = m_object01.start_operation();
    print(PRI1, "coroutine5a(): int v = co_await op;\n");
    int v = co_await op;
    print(PRI1, "coroutine5a(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> Class1430a::coroutine5b()
{
    print(PRI1, "coroutine5b(): async_operation<int> op = m_object02.start_operation();\n");
    async_operation<int> op = m_object02.start_operation();
    print(PRI1, "coroutine5b(): int v = co_await op;\n");
    int v = co_await op;
    print(PRI1, "coroutine5b(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> Class1430a::coroutine4()
{
    print(PRI1, "coroutine4(): async_task<int> a = coroutine5a();\n");
    async_task<int> a = coroutine5a();
    print(PRI1, "coroutine4(): async_task<int> b = coroutine5b();\n");
    async_task<int> b = coroutine5b();
    print(PRI1, "coroutine4(): when_any wa(a, b);\n");
    when_any wa(a, b);
    int idx = -1;
    int v1 = 0;
    int v2 = 0;
    for (int i = 0; i < 2; i++)
    {
        print(PRI1, "coroutine4(): idx = co_await wa;\n");
        idx = co_await wa;
        switch (idx)
        {
        case 0:
        {
            try {
                print(PRI1, "coroutine4(): int v1 = a.get_result();\n");
                v1 = a.get_result();
            }
            catch (const std::system_error& ex) {
                print(PRI1, "coroutine4(): v1 = a.get_result(); raised system_error exception!\n");
            }
            catch (...) {
                print(PRI1, "coroutine4(): v1 = a.get_result(); raised ... exception!\n");
            }
            break;
        }
        case 1:
        {
            try {
                print(PRI1, "coroutine4(): v2 = b.get_result();\n");
                v2 = b.get_result();
            }
            catch (const std::system_error& ex) {
                print(PRI1, "coroutine4(): v2 = b.get_result(); raised system_error exception!\n");
            }
            catch (...) {
                print(PRI1, "coroutine4(): v2 = b.get_result(); raised ... exception!\n");
            }
            break;
        }
        default: print(PRI1, "coroutine4(): co_await wa returned invalid value %d\n", idx); break;
        }
    }
   
    print(PRI1, "coroutine4(): co_return v1 + v2 + 1 = %d;\n", v1 + v2 + 1);
    co_return v1 + v2 + 1;
}

async_task<int> Class1430a::coroutine3()
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

async_task<int> Class1430a::coroutine2()
{
    print(PRI1, "coroutine2(): async_task<int> a = coroutine3();\n");
    async_task<int> a = coroutine3();
    print(PRI1, "coroutine2(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine2(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> Class1430a::coroutine1() {
    print(PRI1, "coroutine1(): async_task<int> a = coroutine2();\n");
    async_task<int> a = coroutine2();
    print(PRI1, "coroutine1(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}
