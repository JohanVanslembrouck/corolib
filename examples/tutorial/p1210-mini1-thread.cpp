/**
 * @file p1210-mini1-thread.cpp
 * @brief
 *
 * Uses a dedicated awaitable type (mini1).
 * A thread launched by coroutine5 calls set_and_resume() on a mini1 object after a delay of 1 second.
 * This call resumes the coroutine that co_waits the mini1 object, which is coroutine5 itself.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <thread>

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "mini1.h"

async_task<int> coroutine5()
{
    print(PRI1, "coroutine5()\n");
    mini1<int> m;
    
    std::thread thread1([&]() {
        print(PRI1, "thread1: std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        print(PRI1); print(PRI1, "thread1: m.set_and_resume(42);\n");
        m.set_and_resume(1);
    });
    thread1.detach();
    
    print(PRI1, "coroutine5(): co_await m;\n");
    int v = co_await m;
    print(PRI1, "coroutine5(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

async_task<int> coroutine4()
{
    print(PRI1, "coroutine4(): async_task<int> a = coroutine5();\n");
    async_task<int> a = coroutine5();
    print(PRI1, "coroutine4(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine4(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

async_task<int> coroutine3()
{
    print(PRI1, "coroutine3(): async_task<int> a1 = coroutine4();\n");
    async_task<int> a1 = coroutine4();
    print(PRI1, "coroutine3(): int v1 = co_await a1;\n");
    int v1 = co_await a1;

    print(PRI1);
    print(PRI1, "coroutine3(): async_task<int> a2 = coroutine4();\n");
    async_task<int> a2 = coroutine4();
    print(PRI1, "coroutine3(): int v2 = co_await a2;\n");
    int v2 = co_await a2;

    print(PRI1);
    print(PRI1, "coroutine3(): co_return v1+v2+1 = %d;\n", v1+v2+1);
    co_return v1+v2+1;
}

async_task<int> coroutine2()
{
    print(PRI1, "coroutine2(): async_task<int> a = coroutine3();\n");
    async_task<int> a = coroutine3();
    print(PRI1, "coroutine2(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine2(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

async_task<int> coroutine1()
{
    print(PRI1, "coroutine1(): async_task<int> a = coroutine2();\n");
    async_task<int> a = coroutine2();
    print(PRI1, "coroutine1(): int v = co_await a;\n");
    int v = co_await a;
    print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        print(PRI1, "main(): async_task<int> a = coroutine1();\n");
        async_task<int> a = coroutine1();
        print(PRI1, "main(): int v = a.get_result();\n");
        int v = a.get_result();
        print(PRI1, "main(): v = %d\n", v);
    }

    print(PRI1, "main(): return 0;\n");
    return 0;
}
