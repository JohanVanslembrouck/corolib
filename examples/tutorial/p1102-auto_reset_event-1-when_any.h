/**
 * @file p1102-auto_reset_event-1-when_any.h
 * @brief
 * Uses 1 auto_reset_event object that will be resumed from main().
 * Coroutine coroutine4 is co_awaited in coroutine3 using when_any.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1102_H_
#define _P1102_H_

#include <functional>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/auto_reset_event.h>
#include <corolib/when_any.h>

using namespace corolib;

extern auto_reset_event are1;

class Class1102
{
public:
    async_task<int> coroutine4()
    {
        print(PRI1, "coroutine4(): co_await are1;\n");
        co_await are1;
        print(PRI1, "coroutine4(): co_return 1;\n");
        co_return 1;
    }

    async_task<int> coroutine3()
    {
        int v = 0;

        print(PRI1, "coroutine3(): async_task<int> a = coroutine4();\n");
        async_task<int> a = coroutine4();
        print(PRI1, "coroutine3(): when_any wa(a);\n");
        when_any wa(a);

        for (int j = 0; j < 1; ++j)
        {
            print(PRI1, "coroutine3(): int i = co_await wa;\n");
            int i = co_await wa;
            print(PRI1, "coroutine3(): i = %d\n", i);
            switch (i) {
            case 0:
                print(PRI1, "coroutine3(): v1 = a.get_result();\n");
                v = a.get_result();
                break;
            default:
                print(PRI1, "coroutine3(): invalid index = %d\n", i);
            }
        }

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
};

#endif
