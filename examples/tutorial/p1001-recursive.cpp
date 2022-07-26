/** 
 * @file p1001-recursive.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

async_task<int> coroutine1(int cntr)
{
    int v = 1;
    if (cntr > 0)
    {
        print(PRI1, "coroutine1(%d): async_task<int> a = coroutine1(%d);\n", cntr, cntr-1);
        async_task<int> a = coroutine1(cntr-1);
        print(PRI1, "coroutine1(%d): int v = co_await a;\n", cntr);
        v = co_await a;
    }
    print(PRI1, "coroutine1(%d): co_return v+1 = %d;\n", cntr, v+1);
    co_return v+1;
}

/**
 * @brief Because main() cannot be a coroutine (it cannot return a coroutine type),
 * it cannot use co_await. Instead it calls get_result() on the coroutine object
 * returned from coroutine1().
 */
int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    print(PRI1, "main(): async_task<int> a = coroutine1(5);\n");
    async_task<int> a = coroutine1(5);
    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);
    print(PRI1, "main(): return 0;\n");
    return 0;
}
