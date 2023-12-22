/**
 * @file p1120-auto_reset_event-oneway_task.cpp
 * @brief
 * Example with 6 coroutines.
 * coroutineI (I = 1..6) co_awaits coroutineI+1.
 *
 * coroutine5 starts coroutine6 twice. 
 * Both instances proceed independently when "someone" resumes the auto_reset_event object passed as the second argument.
 * Each coroutine6 instance has to be resumed twice before coroutine6 will resume 
 * the auto_reset_event passed in its third argument.
 * The main() function is responsible for resuming both instances of coroutine6 twice.
 * The order in which this happens is not important.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/auto_reset_event.h>
#include <corolib/async_task.h>
#include <corolib/oneway_task.h>

using namespace corolib;

auto_reset_event m1, m1a, m2, m2a;

oneway_task coroutine6(const char* name, auto_reset_event &m, auto_reset_event& ma)
{
    print(PRI1, "coroutine6(%s, ...): co_await m;\n", name);
    co_await m;
    print(PRI1, "coroutine6(%s, ...): co_await m;\n", name);
    co_await m;
    print(PRI1, "coroutine6(%s, ...): ma.resume();\n", name);
    ma.resume();
    print(PRI1, "coroutine6(%s, ...): return;\n", name);
}

async_task<int> coroutine5()
{
    print(PRI1, "coroutine5(): coroutine6(\"m1\", m1, m1a);\n");
    (void) coroutine6("m1", m1, m1a);
    print(PRI1, "coroutine5(): coroutine6(\"m2\", m2, m2a);\n");
    (void) coroutine6("m2", m2, m2a);

    print(PRI1, "coroutine5(): co_await m1a;\n");
    co_await m1a;
    print(PRI1, "coroutine5(): co_await m2a;\n");
    co_await m2a;

    print(PRI1, "coroutine5(): int v = 1;\n");
    int v = 1;
    print(PRI1, "coroutine5(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

async_task<int> coroutine4()
{
    print(PRI1, "coroutine4(): async_task<int> a5 = coroutine5();\n");
    async_task<int> a5 = coroutine5();
    print(PRI1, "coroutine4(): int v = co_await a5;\n");
    int v = co_await a5;
    print(PRI1, "coroutine4(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

async_task<int> coroutine3()
{
    print(PRI1, "coroutine3(): async_task<int> a4 = coroutine4();\n");
    async_task<int> a4 = coroutine4();
    print(PRI1, "coroutine3(): int v = co_await a4;\n");
    int v = co_await a4;
    print(PRI1, "coroutine3(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

async_task<int> coroutine2()
{
    print(PRI1, "coroutine2(): async_task<int> a3 = coroutine3();\n");
    async_task<int> a3 = coroutine3();
    print(PRI1, "coroutine2(): int v = co_await a3;\n");
    int v = co_await a3;
    print(PRI1, "coroutine2(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

async_task<int> coroutine1()
{
    print(PRI1, "coroutine1(): async_task<int> a2 = coroutine2();\n");
    async_task<int> a2 = coroutine2();
    print(PRI1, "coroutine1(): int v = co_await a2;\n");
    int v = co_await a2;
    print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    print(PRI1, "main(): async_task<int> a1 = coroutine1();\n");
    async_task<int> a1 = coroutine1();

    print(); print(PRI1, "main(): m2.resume();\n");
    m2.resume();
    print(); print(PRI1, "main(): m1.resume();\n");
    m1.resume();
    print(); print(PRI1, "main(): m1.resume();\n");
    m1.resume();
    print(); print(PRI1, "main(): m2.resume();\n");
    m2.resume();

    print(); print(PRI1, "main(): int v = a1.get_result();\n");
    int v = a1.get_result();
    print(PRI1, "main(): v = %d\n", v);
    print(PRI1, "main(): return 0;\n");
    return 0;
}
