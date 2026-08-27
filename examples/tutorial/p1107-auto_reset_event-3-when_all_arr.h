/**
 * @file p1107-auto_reset_event-3-when_all_arr.h
 * @brief
 * Uses 3 auto_reset_event objects that will be resumed from main().
 * Coroutines coroutine4a, coroutine4b and coroutine4c are co_awaited in coroutine3 using when_all.
 * This example uses
 *      template<typename AsyncBaseType, int Size>
 *      when_all(std::array<AsyncBaseType, Size>& async_ops)
 *
 * @author Johan Vanslembrouck
 */

#ifndef _P1107_ARR_H_
#define _P1107_ARR_H_

#include <array>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/auto_reset_event.h>
#include <corolib/when_all.h>

using namespace corolib;

extern auto_reset_event are1;
extern auto_reset_event are2;
extern auto_reset_event are3;

class Class1107arr
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
        // Special test: nothing to await
        print(PRI1, "coroutine3(): std::array<async_task<int>, 0> arr0;\n");
        std::array<async_task<int>, 0> arr0;
        print(PRI1, "coroutine3(): when_all wa0(arr0);\n");
        when_all wa0(arr0);

        std::array<async_task<int>, 3> arr;
        print(PRI1, "coroutine3(): arr[0] = coroutine4a();\n");
        arr[0] = coroutine4a();
        print(PRI1, "coroutine3(): arr[1] = coroutine4b();\n");
        arr[1] = coroutine4b();
        print(PRI1, "coroutine3(): arr[2] = coroutine4c();\n");
        arr[2] = coroutine4c();

        print(PRI1, "coroutine3(): when_all wa(arr);\n");
        when_all wa(arr);
        print(PRI1, "coroutine3(): co_await wa;\n");
        co_await wa;

        print(PRI1, "coroutine3(): int v1 = arr[0].get_result();\n");
        int v1 = arr[0].get_result();
        print(PRI1, "coroutine3(): int v2 = arr[1].get_result();\n");
        int v2 = arr[1].get_result();
        print(PRI1, "coroutine3(): int v3 = arr[2].get_result();\n");
        int v3 = arr[2].get_result();
        
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
