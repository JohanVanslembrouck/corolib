/**
 * @file p1108-auto_reset_event-3-when_any_vec.h
 * @brief
 * Uses 3 auto_reset_event objects that will be resumed from main().
 * Coroutines coroutine4a, coroutine4b and coroutine4c are co_awaited in coroutine3 using when_any.
 * This example uses
 *      template<typename AsyncBaseType>
 *      when_any(std::vector<AsyncBaseType>& async_ops)
 * 
 * @author Johan Vanslembrouck
 */

#ifndef _P1108_VEC_H_
#define _P1108_VEC_H_

#include <vector>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/auto_reset_event.h>
#include <corolib/when_any.h>

using namespace corolib;

extern auto_reset_event are1;
extern auto_reset_event are2;
extern auto_reset_event are3;

class Class1108vec
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

        // Special test: nothing to await
        print(PRI1, "coroutine3(): std::vector<async_task<int>> vec0;\n");
        std::vector<async_task<int>> vec0(0);
        print(PRI1, "coroutine3(): when_any wa0(vec0);\n");
        when_any wa0(vec0);

        std::vector<async_task<int>> vec(3);
        print(PRI1, "coroutine3(): vec[0] = coroutine4a();\n");
        vec[0] = coroutine4a();
        print(PRI1, "coroutine3(): vec[1] = coroutine4b();\n");
        vec[1] = coroutine4b();
        print(PRI1, "coroutine3(): vec[2] = coroutine4c();\n");
        vec[2] = coroutine4c();

        print(PRI1, "coroutine3(): when_any wa(vec);\n");
        when_any wa(vec);

        for (int j = 0; j < 3; ++j)
        {
            print(PRI1, "coroutine3(): i = co_await wa;\n");
            int i = co_await wa;
            print(PRI1, "coroutine3(): i = %d\n", i);
            switch (i) {
            case 0:
                print(PRI1, "coroutine3(): v1 = vec[0].get_result();\n");
                v1 = vec[0].get_result();
                break;
            case 1:
                print(PRI1, "coroutine3(): v2 = vec[1].get_result();\n");
                v2 = vec[1].get_result();
                break;
            case 2:
                print(PRI1, "coroutine3(): v3 = vec[2].get_result();\n");
                v3 = vec[2].get_result();
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
