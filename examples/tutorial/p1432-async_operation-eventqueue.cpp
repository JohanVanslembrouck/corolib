/**
 *  Filename: p1432-async_operation-eventqueue.cpp
 *  Description:
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 *
 */

#include <functional>

#include <corolib/print.h>
#include <corolib/auto_reset_event.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/wait_all_awaitable.h>

using namespace corolib;

#include "class01.h"

Class01 object01;
Class01 object02;

async_task<int> coroutine5a()
{
	print(PRI1, "coroutine5a(): async_operation<int> op = object01.start_operation();\n");
	async_operation<int> op = object01.start_operation();
	print(PRI1, "coroutine5a(): int v = co_await op;\n");
	int v = co_await op;
	print(PRI1, "coroutine5a(): co_return v+1 = %d;\n", v+1);
	co_return v+1;
}

async_task<int> coroutine5b()
{
	print(PRI1, "coroutine5b(): async_operation<int> op = object02.start_operation();\n");
	async_operation<int> op = object02.start_operation();
	print(PRI1, "coroutine5b(): int v = co_await op;\n");
	int v = co_await op;
	print(PRI1, "coroutine5b(): co_return v+1 = %d;\n", v+1);
	co_return v+1;
}

async_task<int> coroutine4()
{
	print(PRI1, "coroutine4(): async_task<int> a = coroutine5a();\n");
	async_task<int> a = coroutine5a();
	print(PRI1, "coroutine4(): async_task<int> b = coroutine5b();\n");
	async_task<int> b = coroutine5b();
	print(PRI1, "coroutine4(): wait_all_awaitable<async_task<int>> wa({ &a, &b });\n");
	wait_all_awaitable<async_task<int>> wa({ &a, &b });
	print(PRI1, "coroutine4(): co_await wa;\n");
	co_await wa;
	print(PRI1, "coroutine4(): int v = a.get_result() + b.get_result();\n");
	int v = a.get_result() + b.get_result();
	print(PRI1, "coroutine4(): co_return v+1 = %d;\n", v+1);
	co_return v+1;
}

async_task<int> coroutine3()
{
	print(PRI1, "coroutine3(): async_task<int> a1 = coroutine4();\n");
	async_task<int> a1 = coroutine4();
	print(PRI1, "coroutine3(): int v = co_await a1;\n");
	int v1 = co_await a1;

	print();
	print(PRI1, "coroutine3(): async_task<int> a2 = coroutine4();\n");
	async_task<int> a2 = coroutine4();
	print(PRI1, "coroutine3(): int v = co_await a2;\n");
	int v2 = co_await a2;

	print();
	print(PRI1, "coroutine3(): co_return v1+v2+1 = %d;\n", v1+v2+1);
	co_return v1+v2+1;
}

async_task<int> coroutine2()
{
	print(PRI1, "coroutine2(): async_task<int> a = coroutine3();\n");
	async_task<int> a = coroutine3();
	print(PRI1, "coroutine2(): int v = co_await a;\n");
	int v = co_await a;
	print(PRI1, "coroutine2(): co_return v+1 = %d;\n", v+1);
	co_return v+1;
}

async_task<int> coroutine1() {
	print(PRI1, "coroutine1(): async_task<int> a = coroutine2();\n");
	async_task<int> a = coroutine2();
	print(PRI1, "coroutine1(): int v = co_await a;\n");
	int v = co_await a;

	print();
	print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v+1);
	co_return v+1;
}

int main()
{
	print(PRI1, "main(): async_task<int> a = coroutine1();\n");
	async_task<int> a = coroutine1();

	print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	print(PRI1, "main(): before object01.operation(10);\n");
	object01.operation(10);
	print(PRI1, "main(): after object01.operation(10);\n");

	print(PRI1, "main(): before object02.operation(10);\n");
	object02.operation(10);
	print(PRI1, "main(): after object02.operation(10);\n");

	print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	print(PRI1, "main(): before object02.operation(10);\n");
	object02.operation(10);
	print(PRI2, "main(): after object02.operation(10);\n");

	print(PRI1, "main(): before object01.operation(10);\n");
	object01.operation(10);
	print(PRI1, "main(): after object01.operation(10);\n");

	print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	print(PRI1, "main(): int v = a.get_result();\n");
	int v = a.get_result();
	print(PRI1, "main(): v = %d\n", v);

	print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	print(PRI1, "main(): return 0;\n");
	return 0;
}
