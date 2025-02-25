/** 
 * @file p1011-thread-recursive.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <thread>

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

/**
 * @brief coroutine5() starts a thread and then co_wait's a coroutine object
 * created at its beginning.
 * The thread starts a timer and resumes coroutine5() when the timer expires.
 * This implementation uses implementation details from async_task<T> and
 * is therefore more a hack.
 */
async_task<int> coroutine5()
{
    print(PRI1, "coroutine5()\n");
    
#if 1
    // If this section is compiled out,
    // then the behavior of the program is the same as if
    // "ordinary" functions were used.
    
    async_task<int>::promise_type p;
    p.initial_suspend();    // without this statement, m_async_task.coro.done() in await_ready() returns 1 instead of 0
    async_task<int> a = p.get_return_object();

    std::thread thread1([&]() {
        print(PRI1, "thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        print(PRI1);
        print(PRI1, "thread1: p.set_return_value(1);\n");
        p.return_value(1);
        p.final_suspend();
        });
    thread1.detach();

    print(PRI1, "coroutine5(): int v = co_await a;\n");
    int v = co_await a;
#else
    int v = 1;
#endif
    print(PRI1, "coroutine5(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

async_task<int> coroutine1(int cntr)
{
    int v = 0;
    if (cntr > 0)
    {
        print(PRI1, "coroutine1(%d): async_task<int> a = coroutine1(%d);\n", cntr, cntr-1);
        async_task<int> a = coroutine1(cntr-1);
        print(PRI1, "coroutine1(%d): int v = co_await a;\n", cntr);
        v = co_await a;
    }
    else
    {
        print(PRI1, "coroutine1(%d): async_task<int> a = coroutine5();\n", cntr);
        async_task<int> a = coroutine5();
        print(PRI1, "coroutine(%d): int v = co_await a;\n", cntr);
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
    print(PRI1, "main(): int v = awa.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): return 0;\n");
    return 0;
}
