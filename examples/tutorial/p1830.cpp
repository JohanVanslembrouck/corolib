/**
 * @file p1830.cpp
 * @brief
 *
 * 
 * @author Johan Vanslembrouck
 */

#include "p1830.h"
#include "class00.h"

using namespace corolib;

void Class1830::start_operation_impl()
{
    print(PRI1, "start_operation_impl()\n");
    async_op(
        [this](int i)
        {
            print(PRI1, "completionHandler(): op.set_result_and_complete(%d)\n", i);
            op.set_result_and_complete(i);
        });
}

async_task<int> Class1830::coroutine3()
{
    print(PRI1, "coroutine3()\n");
    op3.auto_reset(true);
    std::optional<int> v1 = 0;
    int v = 0;

    do
    {
        print(PRI1, "coroutine3(): v1 = co_await op3;\n");
        v1 = co_await op3;
        print(PRI1, "coroutine3(): op4.set_result_and_complete(v1);\n");
        // Nothing ever co_awaits op4!
        op4.set_result_and_complete(v1);
        if (v1 != std::nullopt)
        {
            print(PRI1, "coroutine1(): v1 = %d\n", v1);
            v += *v1;
        }
        else
            break;
    } while (true);

    print(PRI1, "coroutine3(): co_return v + 1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> Class1830::coroutine2()
{
    print(PRI1, "coroutine2()\n");
    op2.auto_reset(true);
    std::optional<int> v1 = 0;
    int v = 0;

    async_task<int> a3 = coroutine3();

    do
    {
        print(PRI1, "coroutine2(): v1 = co_await op2;\n");
        v1 = co_await op2;
        print(PRI1, "coroutine2(): op3.set_result_and_complete(v1);\n");
        op3.set_result_and_complete(v1);
        if (v1 != std::nullopt)
            v += *v1;
        else
            break;
    } while (true);

    int v2 = co_await a3;

    print(PRI1, "coroutine2(): co_return v + v2 + 1 = %d;\n", v + v2 + 1);
    co_return v + v2 + 1;
}

async_task<int> Class1830::coroutine1()
{
    print(PRI1, "coroutine1()\n");
    op.auto_reset(true);
    std::optional<int> v1 = 0;
    int v = 0;

    async_task<int> a2 = coroutine2();

    do
    {
        print(PRI1, "coroutine1(): v1 = co_await op;\n");
        v1 = co_await op;
        print(PRI1, "coroutine1(): op2.set_result_and_complete(v1);\n");
        op2.set_result_and_complete(v1);
        if (v1 != std::nullopt)
            v += *v1;
        else
            break;
    } while (true);

    int v2 = co_await a2;

    print(PRI1, "coroutine1(): co_return v + v2 + 1 = %d;\n", v + v2 + 1);
    co_return v + v2 + 1;
}
