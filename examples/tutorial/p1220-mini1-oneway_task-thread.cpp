/**
 * @brief p1210-mini1-oneway_task-thread.cpp
 * @brief
 *
 * Uses a dedicated awaitable type (mini1).
 * A thread launched by coroutine6 calls set_and_resume() on a mini1 object after a delay of 1 second.
 * This call resumes the coroutine that co_waits the mini1 object, which is coroutine6 itself.
 *
 * In contrast to all other examples, coroutine3 returns a oneway_task object
 * instead of an async_task<int> object.
 * Therefore, coroutine2 cannot co_await coroutine3.
 * 1) Instead, coroutine2 waits for 10 seconds before it cancels the execution of coroutine3
 * and the coroutines called from coroutine3.
 * 2) Alternatively, coroutine2 can co_wait an auto_reset_event object that it passes to coroutine3
 * coroutine3 will resume coroutine2 using this auto_reset_event object.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <thread>

#include <corolib/print.h>
#include <corolib/auto_reset_event.h>
#include <corolib/async_task.h>
#include <corolib/oneway_task.h>

using namespace corolib;

#include "mini1.h"

async_task<int> coroutine6()
{
    print(PRI1, "coroutine6()\n");
    mini1<int> m;
    
    std::thread thread1([&]() {
        print(PRI1, "thread1: std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        print(); print(PRI1, "thread1: m.set_and_resume(1);\n");
        m.set_and_resume(1);
    });
    thread1.detach();
    
    print(PRI1, "coroutine6(): int v = co_await m;\n");
    int v = co_await m;
    print(PRI1, "coroutine6(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

async_task<int> coroutine5()
{
    print(PRI1, "coroutine5(): async_task<int> a6 = coroutine6();\n");
    async_task<int> a6 = coroutine6();
    print(PRI1, "coroutine5(): int v = co_await a6;\n");
    int v = co_await a6;
    print(PRI1, "coroutine5(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

/**
 * @brief coroutine4 uses a loop where it co_waits coroutine5.
 * coroutine4 leaves the loop before the maximum number of 30 iterations
 * if the value of cancel parameter changes from false to true.
 * coroutine2 will change this value after 10 seconds.
 * If not, coroutine4 will leave the loop after 30 iterations and 30 seconds.
 * @param cancel is a bool that allows another coroutine or function to exit
 * the for loop before the maximum number of iterations.
 * @return async_task<int>
 */
async_task<int> coroutine4(bool& cancel)
{
    int v = 0;
    for (int i = 0; i < 30 && !cancel; i++)
    {
        print(PRI1, "coroutine4(): async_task<int> a5 = coroutine5();\n");
        async_task<int> a5 = coroutine5();
        print(PRI1, "coroutine4(): v += co_await a5;\n");
        v += co_await a5;
    }

    print();
    print(PRI1, "coroutine4(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

oneway_task coroutine3(const char* name, auto_reset_event& ma, bool &cancel)
{
    print(PRI1, "coroutine3(): async_task<int> a4 = coroutine4();\n");
    async_task<int> a4 = coroutine4(cancel);
    print(PRI1, "coroutine3(): co_await cor13;\n");
    co_await a4;
    print(PRI1, "coroutine3(): ma.resume();\n");
    ma.resume();
}

async_task<int> coroutine2()
{
    auto_reset_event m1;
    bool cancel = false;

    print(PRI1, "coroutine2(): coroutine3(\"m1\", m1, cancel);\n");
    /* oneway_task a3 = */ coroutine3("m1", m1, cancel);

#if 1
    print(PRI1, "coroutine2: before std::this_thread::sleep_for(std::chrono::milliseconds(10000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    print(PRI1, "coroutine2: after std::this_thread::sleep_for(std::chrono::milliseconds(10000));\n");
    cancel = true;
#else
    print(PRI1, "coroutine2(): co_await m1;\n");
    co_await m1;
#endif

    print(PRI1, "coroutine2(): int v = 1;\n");
    int v = 1;
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

    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);
        print(PRI1, "main(): async_task<int> a1 = coroutine1();\n");
        async_task<int> a1 = coroutine1();
        print(); print(PRI1, "main(): int v = a1.get_result();\n");
        int v = a1.get_result();
        print(PRI1, "main(): v = %d\n", v);
    }

    print(PRI1, "main(): return 0;\n");
    return 0;
}
