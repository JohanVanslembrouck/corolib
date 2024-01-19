/**
 * @file p1100-auto_reset_event-1.cpp
 * @brief
 * Uses 1 auto_reset_event object that will be resumed from main().
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <thread>

#include <corolib/print.h>
#include <corolib/auto_reset_event.h>
#include <corolib/async_task.h>

using namespace corolib;

auto_reset_event are1;

async_task<int> coroutine4()
{
    print(PRI1, "coroutine4(): co_await are1;\n");
    co_await are1;
    print(PRI1, "coroutine4(): co_return 1;\n");
    co_return 1;
}

async_task<int> coroutine3()
{
    print(PRI1, "coroutine3(): int v = co_await coroutine4();\n");
    int v = co_await coroutine4();
    print(PRI1, "coroutine3(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> coroutine2()
{
    print(PRI1, "coroutine2(): int v = co_await coroutine3();\n");
    int v = co_await coroutine3();
    print(PRI1, "coroutine2(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

async_task<int> coroutine1()
{
    print(PRI1, "coroutine1(): int v = co_await coroutine2();\n");
    int v = co_await coroutine2();
    print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    print(PRI1, "main(): async_task<int> a = coroutine1();\n");
    async_task<int> a = coroutine1();
	
    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    print(PRI1, "main(): are1.resume();\n");
    are1.resume();
	
    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);
    print(PRI1, "main(): return 0;\n");
    return 0;
}
