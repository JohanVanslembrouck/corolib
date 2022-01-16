/**
 *  Filename: p1200-mini0.cpp
 *  Description:
 *
 *		Uses a dedicated coroutine type (mini0).
 *
 *		An global object of this type is co_awaited in coroutine5 and
 * 		is resumed from main().
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

#include "mini0.h"

mini0 m;

async_task<int> coroutine5()
{
	print(PRI1, "coroutine5()\n");
	print(PRI1, "coroutine5(): co_await m;\n");
	co_await m;
	int v = 1;
	print(PRI1, "coroutine5(): co_return v+1 = %d;\n", v+1);
	co_return v+1;
}

async_task<int> coroutine4()
{
	print(PRI1, "coroutine4(): async_task<int> a = coroutine5();\n");
	async_task<int> a = coroutine5();
	print(PRI1, "coroutine4(): int v = co_await a;\n");
	int v = co_await a;
	print(PRI1, "coroutine4(): co_return v+1 = %d;\n", v+1);
	co_return v+1;
}

async_task<int> coroutine3()
{
	print(PRI1, "coroutine3(): async_task<int> a1 = coroutine4();\n");
	async_task<int> a1 = coroutine4();
	print(PRI1, "coroutine3(): int v1 = co_await a1;\n");
	int v1 = co_await a1;

	print();
	print(PRI1, "coroutine3(): async_task<int> a2 = coroutine4();\n");
	async_task<int> a2 = coroutine4();
	print(PRI1, "coroutine3(): int v2 = co_await a2;\n");
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

async_task<int> coroutine1()
{
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

	print(PRI1, "main(): before m.resume();\n");
	m.resume();
	print(PRI1, "main(): after m.resume();\n");

	print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	print(PRI1, "main(): before m.resume();\n");
	m.resume();
	print(PRI1, "main(): after m.resume();\n");

	print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

#if 0
	print(PRI1, "main(): before m.resume();\n");
	m.resume();
	print(PRI1, "main(): after m.resume();\n");

	print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	print(PRI1, "main(): before m.resume();\n");
	m.resume();
	print(PRI1, "main(): after m.resume();\n");

	print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	print(PRI1, "main(): before m.resume();\n");
	m.resume();
	print(PRI1, "main(): after m.resume();\n");

	print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
#endif

	print(PRI1, "main(): int v = a.get();\n");
	int v = a.get();
	print(PRI1, "main(): v = %d\n", v);

	print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	print(PRI1, "main(): return 0;\n");
	return 0;
}
