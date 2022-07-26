/**
 * @file p1105-auto_reset_event.cpp
 * @brief
 * Example with two coroutines coroutine1 and coroutine2 and 
 * awaitable type auto_reset_event from corolib.
 *
 * This example is a variant of p1100-auto_reset_event.cpp.
 * coroutine2 co_awaits a global auto_reset_event object are1.
 * Because the object is not yet ready at this point, 
 * coroutine2 suspends and returns control to coroutine1.
 * coroutine1 (instead of main) will resume coroutine2 via are1.
 * To be able to do this, coroutine1 cannot use co_await directly on coroutine2,
 * but it has to save the return value of coroutine2 in a local variable.
 * 
 * This a an example of the caller (coroutine1) of a coroutine (coroutine2)
 * that resumes the coroutine it has first called.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/auto_reset_event.h>
#include <corolib/async_task.h>

using namespace corolib;

auto_reset_event are1;

async_task<int> coroutine2()
{
    print(PRI1, "coroutine2(): co_await are1;\n");
    co_await are1;
    print(PRI1, "coroutine2(): co_return 1;\n");
    co_return 1;
}

async_task<int> coroutine1()
{
    print(PRI1, "coroutine1(): int v = co_await coroutine2();\n");
    async_task<int> at = coroutine2();

    print(PRI1, "coroutine1(): are1.resume();\n");
    are1.resume();
	
	print(PRI1, "coroutine1(): int v = co_await at;\n");
	int v = co_await at;
	
    print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v+1);
    co_return v+1;
}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    print(PRI1, "main(): async_task<int> a = coroutine1();\n");
    async_task<int> a = coroutine1();

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);
    return 0;
}
