/**
 * @file p1470.cpp
 * @brief
 * Example with 6 coroutines.
 * coroutineI (I = 1..4) co_awaits coroutineI+1.
 * In contrast to most other p14X0.cpp files, coroutine3 calls coroutine4 only once.
 * coroutine4 starts coroutine5a, coroutine5b and coroutine5c and awaits their completion.
 * coroutine5a and coroutine5b are very similar and run a potentially infinite loop using
 * a variable "running" that is initialized to true.
 * coroutine5c starts a thread that sleeps for 30 seconds.
 * When the thread returns from its sleep, coroutine5c continues and sets "running" to false.
 * coroutine5a, coroutine5b and coroutine5c co_return at this moment.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/when_all.h>
#include <corolib/auto_reset_event.h>

using namespace corolib;

#include "p1470.h"

async_task<int> Class1470::coroutine5a()
{
    print(PRI1, "coroutine5a()\n");
    int v = 0;
    while (m_running)
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

async_task<int> Class1470::coroutine5b()
{
    print(PRI1, "coroutine5b()\n");
    int v = 0;
    while (m_running)
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

async_task<int> Class1470::coroutine5c()
{
    print(PRI1, "coroutine5c()\n");

    auto_reset_event are;

    std::thread thread1([&are]() {
        print(PRI1, "coroutine5c(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(30000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(30000));
        print(PRI1, "coroutine5c(): thread1: are.resume();\n");
        are.resume();
        print(PRI1, "coroutine5c(): thread1: return;\n");
        });
    thread1.detach();

    print(PRI1, "coroutine5c(): co_await are;\n");
    co_await are;

    print(PRI1, "coroutine5c(): m_running = false;\n");
    m_running = false;

    print(PRI1, "coroutine5c(): co_return 0;\n");
    co_return 0;
}

async_task<int> Class1470::coroutine4()
{
    print(PRI1, "coroutine4(): async_task<int> a = coroutine5a();\n");
    async_task<int> a = coroutine5a();
    print(PRI1, "coroutine4(): async_task<int> b = coroutine5b();\n");
    async_task<int> b = coroutine5b();
    print(PRI1, "coroutine4(): async_task<int> c = coroutine5c();\n");
    async_task<int> c = coroutine5c();
    print(PRI1, "coroutine4(): when_all wa(a, b, c);\n");
    when_all wa(a, b, c);
    print(PRI1, "coroutine4(): co_await wa;\n");
    co_await wa;
    print(PRI1, "coroutine4(): int v = a.get_result() + b.get_result();\n");
    int v = a.get_result() + b.get_result();
    print(PRI1, "coroutine4(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> Class1470::coroutine3()
{
    print(PRI1, "coroutine3(): async_task<int> a1 = coroutine4();\n");
    async_task<int> a1 = coroutine4();
    print(PRI1, "coroutine3(): int v = co_await a1;\n");
    int v = co_await a1;
    print(PRI1, "coroutine3(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> Class1470::coroutine2()
{
    print(PRI1, "coroutine2(): async_task<int> a = coroutine3();\n");
    async_task<int> a = coroutine3();
    print(PRI1, "coroutine2(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine2(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> Class1470::coroutine1() {
    print(PRI1, "coroutine1(): async_task<int> a = coroutine2();\n");
    async_task<int> a = coroutine2();
    print(PRI1, "coroutine1(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}
