/**
 * @file p1101-auto_reset_event-1-when_all.h
 * @brief
 * Uses 1 auto_reset_event object that will be resumed from main().
 * Coroutine coroutine4 is co_awaited in coroutine3 using when_all.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1101_H_
#define _P1101_H_

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/auto_reset_event.h>
#include <corolib/when_all.h>

using namespace corolib;

extern auto_reset_event are1;

class Class1101
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
        print(PRI1, "coroutine3(): int v = co_await coroutine4();\n");
        async_task<int> a = coroutine4();
        print(PRI1, "coroutine3(): when_all wa(a);\n");
        when_all wa(a);
        print(PRI1, "coroutine3(): co_await wa;\n");
        co_await wa;
        print(PRI1, "coroutine3(): int v = a.get_result();\n");
        int v = a.get_result();
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
