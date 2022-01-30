/**
 *  Filename: p1210-mini1-oneway_task-thread.cpp
 *  Description:
 * 
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 *
 */

#include <corolib/print.h>
#include <corolib/auto_reset_event.h>
#include <corolib/async_task.h>
#include <corolib/oneway_task.h>

using namespace corolib;

#include "mini1.h"

async_task<int> coroutine6()
{
	print(PRI1, "coroutine6()\n");
	mini1<int> m;
	
	std::thread thread1([&]() {
		print(PRI1, "thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		print(); print(PRI1, "thread1: m.set_and_resume(1);\n");
		m.set_and_resume(1);
	});
	thread1.detach();
	
	print(PRI1, "coroutine6(): int v = co_await m;\n");
	int v = co_await m;
	print(PRI1, "coroutine6(): co_return v+1 = %d;\n", v+1);
	co_return v+1;
}

async_task<int> coroutine5()
{
	print(PRI1, "coroutine5(): async_task<int> a6 = coroutine6();\n");
	async_task<int> a6 = coroutine6();
	print(PRI1, "coroutine5(): int v = co_await a6;\n");
	int v = co_await a6;
	print(PRI1, "coroutine5(): co_return v+1 = %d;\n", v+1);
	co_return v+1;
}

async_task<int> coroutine4(bool& cancel)
{
	int v = 0;
	for (int i = 0; i < 30 && !cancel; i++)
	{
		print(PRI1, "coroutine4(): async_task<int> a5 = coroutine5();\n");
		async_task<int> a5 = coroutine5();
		print(PRI1, "coroutine4(): v += co_await a5;\n");
	    v += co_await a5;
	}

	print();
	print(PRI1, "coroutine4(): co_return v+1 = %d;\n", v+1);
	co_return v+1;
}

oneway_task coroutine3(const char* name, auto_reset_event& ma, bool &cancel)
{
	print(PRI1, "coroutine3(): async_task<int> a4 = coroutine4();\n");
	async_task<int> a4 = coroutine4(cancel);
	print(PRI1, "coroutine3(): co_await cor13;\n");
	co_await a4;
	print(PRI1, "coroutine3(): ma.resume();\n");
	ma.resume();
}

async_task<int> coroutine2()
{
	auto_reset_event m1;
	bool cancel = false;

	print(PRI1, "coroutine2(): coroutine3(\"m1\", m1, cancel);\n");
	/* oneway_task a3 = */ coroutine3("m1", m1, cancel);

#if 1
	print(PRI1, "coroutine2: before std::this_thread::sleep_for(std::chrono::milliseconds(10000));\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	print(PRI1, "coroutine2: after std::this_thread::sleep_for(std::chrono::milliseconds(10000));\n");
	cancel = true;
#else
	print(PRI1, "coroutine2(): co_await m1;\n");
	co_await m1;
#endif

	print(PRI1, "coroutine2(): int v = 1;\n");
	int v = 1;
	print(PRI1, "coroutine2(): co_return v+1 = %d;\n", v+1);
	co_return v+1;
}

async_task<int> coroutine1()
{
	print(PRI1, "coroutine1(): async_task<int> a2 = coroutine2();\n");
	async_task<int> a2 = coroutine2();
	print(PRI1, "coroutine1(): int v = co_await a2;\n");
	int v = co_await a2;
	print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v+1);
	co_return v+1;
}

int main()
{
	print(PRI1, "main(): async_task<int> a1 = coroutine1();\n");
	async_task<int> a1 = coroutine1();
	print(); print(PRI1, "main(): int v = a1.get_result();\n");
	int v = a1.get_result();
	print(PRI1, "main(): v = %d\n", v);
	return 0;
}
