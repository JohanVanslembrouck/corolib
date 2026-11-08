/**
 * @file p1700a.cpp
 * @brief
 * Example with 5 coroutines.
 * coroutineI (I = 1..4) co_awaits coroutineI+1.
 * coroutine3 calls coroutine4 twice.
 * coroutine5 starts an asynchronous operation and awaits its completion.
 *
 * p1700a.cpp is a variant of p1700.cpp that shows that lazy start coroutines can all
 * be started and assigned to async_ltask<int> objects.
 * Afterwards, these objects are co_awaited in the coroutines.
 * For new runs (iterations), the async_ltask<int> objects have to be re-initialized.
 * 
 * Notice, however, that not all final_awaiter objects are destructed
 * before the end of the program:
 * 
 * 00: ---------------------------------------------------------------
 * 00:     cons    dest    diff    max     c>p     c<p     c>h     c<h
 * 00: ope 3       3       0       2
 * 00: cor 6       6       0       1       4       2       6       0
 * 00: pro 6       6       0       1       4       2
 * 00: fin 11      6       5       6                       0       0
 * 00: ---------------------------------------------------------------
 * 
 * This is because these objects are global.
 * 
 * @author Johan Vanslembrouck
 */

#include "p1700.h"
#include "class00.h"

using namespace corolib;

extern async_ltask<int> a5;
extern async_ltask<int> a4;
extern async_ltask<int> a3;
extern async_ltask<int> a2;
extern async_ltask<int> a1;

async_ltask<int> coroutine5()
{
    print(PRI1, "coroutine5(): async_operation<int> op;\n");
    async_operation<int> op;
    print(PRI1, "coroutine5(): start_operation_impl(op);\n");
    start_operation_impl(op);
    print(PRI1, "coroutine5(): int v = co_await op;\n");
    int v = co_await op;
    print(PRI1, "coroutine5(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_ltask<int> coroutine4()
{
    print(PRI1, "coroutine4(): int v = co_await a5;\n");
    int v = co_await a5;
    print(PRI1, "coroutine4(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_ltask<int> coroutine3()
{
    print(PRI1, "coroutine3(): int v1 = co_await a4;\n");
    int v1 = co_await a4;

    print(PRI1);
    print(PRI1, "coroutine3(): int v2 = co_await a4;\n");
    int v2 = co_await a4;

    print(PRI1);
    print(PRI1, "coroutine3(): co_return v1+v2+1 = %d;\n", v1 + v2 + 1);
    co_return v1 + v2 + 1;
}

async_ltask<int> coroutine2()
{
    print(PRI1, "coroutine2(): int v = co_await a3;\n");
    int v = co_await a3;
    print(PRI1, "coroutine2(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_ltask<int> coroutine1()
{
    print(PRI1, "coroutine1(): int v = co_await a2;\n");
    int v = co_await a2;
    print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v + 1);
    co_return v + 1;
}

async_ltask<void> coroutine0()
{
    print(PRI1, "coroutine0(): co_await a1;\n");
    co_await a1;
}

async_ltask<int> a5 = coroutine5();     // a5 is co_awaited in coroutine4
async_ltask<int> a4 = coroutine4();     // a4 is co_awaited in coroutine3
async_ltask<int> a3 = coroutine3();     // a3 is co_awaited in coroutine2
async_ltask<int> a2 = coroutine2();     // a2 is co_awaited in coroutine1
async_ltask<int> a1 = coroutine1();     // a1 is co_awaited in coroutine0

/**
 * @brief reinit reinitializes the async_ltask<int> objects for a new run.
 * 
 */
void reinit()
{
    a5 = coroutine5();
    a4 = coroutine4();
    a3 = coroutine3();
    a2 = coroutine2();
    // a1 = coroutine1();
    // With the previous line present, we see the following output:
    // 00: 00007FF65D8B3A88: async_task_base<TYPE>::destroy_coroutine_frame(): m_coro_handle.done() returned false
    // The reason is that a1 has not been used in the first stage (before the reinit call),
    // meaning that the a1 object has not run to its end.
}