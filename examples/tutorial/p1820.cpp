/**
 * @file p1820.cpp
 * @brief
 *
 * 
 * @author Johan Vanslembrouck
 */

#include "p1820.h"
#include "class00.h"

void Class1820::start_operation_impl()
{
    print(PRI1, "start_operation_impl()\n");
    async_op(
        [this](int i)
        {
            print(PRI1, "completionHandler(): op.set_result_and_complete(%d)\n", i);
            op.set_result_and_complete(i);
        });
}

async_task<int> Class1820::coroutine1()
{
    print(PRI1, "coroutine1(): async_operation<int> op;\n");
    op.auto_reset(true);
    std::optional<int> v1 = 0;
    int v = 0;

    do
    {
        print(PRI1, "coroutine1(): v1 = co_await op;\n");
        v1 = co_await op;
        if (v1 != std::nullopt)
        {
            print(PRI1, "coroutine1(): v1 = %d\n", v1);
            v += *v1;
        }
        else
            break;
    } while (true);

    print(PRI1, "coroutine1(): co_return v + 1 = %d;\n", v + 1);
    co_return v + 1;
}
