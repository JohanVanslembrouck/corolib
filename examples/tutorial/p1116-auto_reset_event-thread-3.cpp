/**
 * @file p1116-auto_reset_event-thread-3.cpp
 * @brief
 * Uses 3 auto_reset_event objects that will be resumed from 3 threads.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <thread>

#include <corolib/print.h>
#include <corolib/auto_reset_event.h>
#include <corolib/async_task.h>

using namespace corolib;

async_task<int> coroutine4a()
{
    auto_reset_event are;

    std::thread thread1([&are]() {
        print(PRI1, "coroutine4a(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        print(PRI1, "coroutine4a(): thread1: are.resume();\n");
        are.resume();
        print(PRI1, "coroutine4a(): thread1: return;\n");
        });
    thread1.detach();

    print(PRI1, "coroutine4a(): co_await are;\n");
    co_await are;

    print(PRI1, "coroutine4a(): co_return 1;\n");
    co_return 1;
}

async_task<int> coroutine4b()
{
    auto_reset_event are;

    std::thread thread1([&are]() {
        print(PRI1, "coroutine4b(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(2000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        print(PRI1, "coroutine4b(): thread1: are.resume();\n");
        are.resume();
        print(PRI1, "coroutine4b(): thread1: return;\n");
        });
    thread1.detach();

    print(PRI1, "coroutine4b(): co_await are;\n");
    co_await are;

    print(PRI1, "coroutine4b(): co_return 2;\n");
    co_return 1;
}

async_task<int> coroutine4c()
{
    auto_reset_event are;

    std::thread thread1([&are]() {
        print(PRI1, "coroutine4c(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(3000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        print(PRI1, "coroutine4c(): thread1: are.resume();\n");
        are.resume();
        print(PRI1, "coroutine4c(): thread1: return;\n");
        });
    thread1.detach();

    print(PRI1, "coroutine4c(): co_await are;\n");
    co_await are;

    print(PRI1, "coroutine4c(): co_return 2;\n");
    co_return 1;
}

async_task<int> coroutine3()
{
    print(PRI1, "coroutine3(): async_task<int> a = coroutine4a();\n");
    async_task<int> a = coroutine4a();
    print(PRI1, "coroutine3(): async_task<int> b = coroutine4b();\n");
    async_task<int> b = coroutine4b();
    print(PRI1, "coroutine3(): async_task<int> c = coroutine4c();\n");
    async_task<int> c = coroutine4c();

    print(PRI1, "coroutine3(): int v1 = co_await a;\n");
    int v1 = co_await a;
    print(PRI1, "coroutine3(): int v2 = co_await b;\n");
    int v2 = co_await b;
    print(PRI1, "coroutine3(): int v2 = co_await c;\n");
    int v3 = co_await c;    
    print(PRI1, "coroutine3(): co_return v+1 = %d;\n", v1 + v2 + v3 + 1);
    co_return v1 + v2 + v3 + 1;
}

async_task<int> coroutine2()
{
    print(PRI1, "coroutine2(): int v = co_await coroutine3;\n");
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
    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);
    print(PRI1, "main(): return 0;\n");
    return 0;
}
