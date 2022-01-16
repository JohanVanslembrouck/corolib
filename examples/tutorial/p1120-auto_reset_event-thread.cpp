/**
 *  Filename: p1120-auto_reset_event-thread.cpp
 *  Description:
 *
 *		This example defines coroutine5 based
 *		on the coroutine extension of future by Microsoft.
 *		See implementation of await_ready(), await_suspend()
 *		and await_resume().
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 *  Based upon: https://kirit.com/How%20C%2B%2B%20coroutines%20work/Awaiting
 *
 */

#include "print.h"
#include "async_task.h"
#include "auto_reset_event.h"
#include "semaphore.h"

using namespace corolib;

async_task<int> coroutine5()
{
	print(PRI1, "coroutine5(): int v = 1\n");
	int v = 1;

	auto_reset_event sync6;

	std::thread thread1([&sync6]() {
		print(PRI1, "coroutine5(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		print(PRI1, "coroutine5(): thread1: sync6.resume();\n");
		sync6.resume();
		print(PRI1, "coroutine5(): thread1: return;\n");
		});
	thread1.detach();

	print(PRI1, "coroutine5(): co_await sync6;\n");
	co_await sync6;

	print(PRI1, "coroutine5(): co_return v+1 = %d;\n", v+1);
	co_return v+1;
}

async_task<int> coroutine4()
{
	print(PRI1, "coroutine4(): 1: async_task<int> sync5 = coroutine5();\n");
	async_task<int> sync5 = coroutine5();
	print(PRI1, "coroutine4(): 2: int v = co_await sync5;\n");
	int v = co_await sync5;
	print(PRI1, "coroutine4(): 3: co_return v+1 = %d;\n", v+1);
	co_return v+1;
}

async_task<int> coroutine3()
{
	print(PRI1, "coroutine3(): 1: async_task<int> sync4 = coroutine4();\n");
	async_task<int> sync4 = coroutine4();
	print(PRI1, "coroutine3(): 2: int v = co_await sync4;\n");
	int v = co_await sync4;
	print(PRI1, "coroutine3(): 3: co_return v+1 = %d;\n", v+1);
	co_return v+1;
}

async_task<int> coroutine2()
{
	int v = 0;
	for (int j = 0; j < 1; j++)
	{
		print(PRI1, "coroutine2(): 1: async_task<int> sync3 = coroutine3();\n");
		async_task<int> sync3 = coroutine3();
		print(PRI1, "coroutine2(): 2: v += co_await sync3;\n");
		v += co_await sync3;
	}
	print(PRI1, "coroutine2(): 3: co_return v+1 = %d;\n", v+1);
	co_return v+1;
}

async_task<int> coroutine1()
{
	print(PRI1, "coroutine1(): 1: async_task<int> sync2 = coroutine2();\n");
	async_task<int> sync2 = coroutine2();
	print(PRI1, "coroutine1(): 2: int v = co_await sync2;\n");
	int v = co_await sync2;
	print(PRI1, "coroutine1(): 3: co_return v+1 = %d;\n", v+1);
	co_return v+1;
}

int main()
{
	print(PRI1, "main(): 1: async_task<int> sync1 = coroutine1();\n");
	async_task<int> sync1 = coroutine1();
	print(PRI1, "main(): 2: int v = sync1.get();\n");
	int v = sync1.get();
	print(PRI1, "main(): 3: v = %d\n", v);
	return 0;
}
