/**
 * @file p1900-resume_in_middle.cpp
 * @brief
 * Example with 6 coroutines.
 * coroutineI (I = 1..5) co_awaits coroutineI+1.
 * Instead of resuming nicely from the top of the call stack (coroutine6),
 * this example resumes at coroutine4 by using the coroutine_handle to this coroutine.
 * This is not the way to do, of course.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/auto_reset_event.h>
#include <corolib/async_task.h>

using namespace corolib;

// No appropriate default constructor available
//async_task<int> a;

auto_reset_event are1;

async_task<int>* tasks[7];

async_task<int> coroutine6()
{
    print(PRI1, "coroutine6(): co_await are1;\n");
    co_await are1;
    print(PRI1, "coroutine6(): co_return 1;\n");
    co_return 1;
}

async_task<int> coroutine5()
{
    print(PRI1, "coroutine5(): coroutine6(\"m1\", m1, m1a);\n");
    async_task<int> a6 = coroutine6();
    tasks[6] = &a6;
    print(PRI1, "coroutine5(): int v = co_await a5;\n");
    int v = co_await a6;
    print(PRI1, "coroutine5(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

async_task<int> coroutine4()
{
    print(PRI1, "coroutine4(): async_task<int> a5 = coroutine5();\n");
    async_task<int> a5 = coroutine5();
    tasks[5] = &a5;
    print(PRI1, "coroutine4(): int v = co_await a5;\n");
    int v = co_await a5;
    print(PRI1, "coroutine4(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

async_task<int> coroutine3()
{
    print(PRI1, "coroutine3(): async_task<int> a4 = coroutine4();\n");
    async_task<int> a4 = coroutine4();
    tasks[4] = &a4;
    print(PRI1, "coroutine3(): int v = co_await a4;\n");
    int v = co_await a4;
    print(PRI1, "coroutine3(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

async_task<int> coroutine2()
{
    print(PRI1, "coroutine2(): async_task<int> a3 = coroutine3();\n");
    async_task<int> a3 = coroutine3();
    tasks[3] = &a3;
    print(PRI1, "coroutine2(): int v = co_await a3;\n");
    int v = co_await a3;
    print(PRI1, "coroutine2(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

async_task<int> coroutine1()
{
    print(PRI1, "coroutine1(): async_task<int> a2 = coroutine2();\n");
    async_task<int> a2 = coroutine2();
    tasks[2] = &a2;
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
    tasks[1] = &a1;
    tasks[0] = nullptr;

    print(); 
    //print(PRI1, "main(): are.resume();\n");
    //are1.resume();
    print(PRI1, "main(): tasks[4]->start();\n");
    tasks[4]->start();
   
    print(); print(PRI1, "main(): int v = a1.get_result();\n");
    int v = a1.get_result();
    print(PRI1, "main(): v = %d\n", v);
    return 0;
}
