/**
 * @file p1108-auto_reset_event-3-when_any.h
 * @brief
 * Uses 3 auto_reset_event objects that will be resumed from main().
 * Coroutines coroutine4a, coroutine4b and coroutine4c are co_awaited in coroutine3 using when_any.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1108_H_
#define _P1108_H_

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/auto_reset_event.h>
#include <corolib/when_any.h>

using namespace corolib;

extern auto_reset_event are1;
extern auto_reset_event are2;
extern auto_reset_event are3;

class Class1108
{
public:
    async_task<int> coroutine4a()
    {
        print(PRI1, "coroutine4a(): co_await are1;\n");
        co_await are1;
        print(PRI1, "coroutine4a(): co_return 1;\n");
        co_return 1;
    }

    async_task<int> coroutine4b()
    {
        print(PRI1, "coroutine4b(): co_await are2;\n");
        co_await are2;
        print(PRI1, "coroutine4b(): co_return 1;\n");
        co_return 2;
    }

    async_task<int> coroutine4c()
    {
        print(PRI1, "coroutine4c(): co_await are3;\n");
        co_await are3;
        print(PRI1, "coroutine4c(): co_return 1;\n");
        co_return 2;
    }

    async_task<int> coroutine3()
    {
        int v1, v2, v3;
        v1 = v2 = v3 = 0;

        print(PRI1, "coroutine3(): async_task<int> a = coroutine4a();\n");
        async_task<int> a = coroutine4a();
        print(PRI1, "coroutine3(): async_task<int> b = coroutine4b();\n");
        async_task<int> b = coroutine4b();
        print(PRI1, "coroutine3(): async_task<int> c = coroutine4c();\n");
        async_task<int> c = coroutine4c();

        print(PRI1, "coroutine3(): when_any wa(a, b, c);\n");
        when_any wa(a, b, c);

        for (int j = 0; j < 3; ++j)
        {
            print(PRI1, "coroutine3(): i = co_await wa;\n");
            int i = co_await wa;
            print(PRI1, "coroutine3(): i = %d\n", i);
            switch (i) {
            case 0:
                print(PRI1, "coroutine3(): v1 = a.get_result();\n");
                v1 = a.get_result();
                break;
            case 1:
                print(PRI1, "coroutine3(): v2 = b.get_result();\n");
                v2 = b.get_result();
                break;
            case 2:
                print(PRI1, "coroutine3(): v3 = c.get_result();\n");
                v3 = c.get_result();
                break;
            default:
                print(PRI1, "coroutine3(): invalid index = %d\n", i);
            }
        }

        print(PRI1, "coroutine3(): co_return v+1 = %d;\n", v1 + v2 + v3 + 1);
        co_return v1 + v2 + v3 + 1;
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
};

#endif
