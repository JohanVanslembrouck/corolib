/**
 *  Filename: p1100-auto_reset_event.cpp
 *  Description:
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 *
 */

#include "print.h"
#include "auto_reset_event.h"
#include "async_task.h"

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
	print(PRI1, "coroutine1(): int i = co_await coroutine2();\n");
	int v = co_await coroutine2();
	print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v+1);
	co_return v+1;
}

int main()
{
	print(PRI1, "main(): awaitable the_coroutine1 = coroutine1();\n");
	async_task<int> the_coroutine1 = coroutine1();
	print(PRI1, "main(): are1.resume();\n");
	are1.resume();
	print(PRI1, "main(): int v = the_coroutine1.get();\n");
	int v = the_coroutine1.get();
	print(PRI1, "main(): v = %d\n", v);
	return 0;
}
