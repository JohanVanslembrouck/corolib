/**
 * @file p1810.cpp
 * @brief
 * 
 * Because async_operation<int> op is co_awaited by two coroutines,
 * both coroutines have to be resumed when op is completed.
 * 
 * @author Johan Vanslembrouck
 */

#include <corolib/when_all.h>

#include "p1800.h"
#include "class00.h"

/**
 * @brief start_operation_impl simulates starting an asynchronous operation
 * 
 * @param op is a reference to an async_operation<int> object.
 */
void start_operation_impl(async_operation_rmc<int>& op)
{
    delay = 0;

    print(PRI1, "start_operation_impl()\n");
    async_op(
        [&op](int i)
        {
            print(PRI1, "completionHandler(): op.set_result_and_complete(%d)\n", i);
            op.set_result_and_complete(i);
        });
}

async_operation_rmc<int> op;
   
async_task<int> coroutine1a()
{
    print(PRI1, "coroutine1a(): async_operation<int> op;\n");
    int v1 = 0;
    int v = 0;
    
    do
    {
        print(PRI1, "coroutine1a(): v1 = co_await op;\n");
        v1 = co_await op;
        v += v1;
    }
    while (v1 != 0);
    
    print(PRI1, "coroutine1a(): co_return v + 1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> coroutine1b()
{
    print(PRI1, "coroutine1b(): async_operation<int> op;\n");
    int v1 = 0;
    int v = 0;
    
    do
    {
        print(PRI1, "coroutine1b(): int v1 = co_await op;\n");
        v1 = co_await op;
        v += v1;
    }
    while (v1 != 0);
    
    print(PRI1, "coroutine1b(): co_return v + 1 = %d;\n", v + 1);
    co_return v + 1;
}

async_task<int> coroutine1()
{
    print(PRI1, "coroutine1(): async_operation<int> op;\n");
    op.auto_reset(true);
   
    async_task<int> t1 = coroutine1a();
    async_task<int> t2 = coroutine1b();
#if 0
	// The following statement does not compile with g++ 11.3.0
    co_await when_all( { &t1, &t2 } );
#else
    co_await when_all(t1, t2);
#endif
    int v = t1.get_result() + t2.get_result();
    
    print(PRI1, "coroutine1(): co_return v + 1 = %d;\n", v + 1);
    co_return v + 1;
}
