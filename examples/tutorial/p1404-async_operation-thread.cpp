/**
 *  Filename: p1404-async_operation-thread.cpp
 *  Description:
 * 
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 *
 */

#include <functional>

#include "print.h"
#include "auto_reset_event.h"
#include "async_task.h"
#include "async_operation.h"

using namespace corolib;

std::function<void(int)> operation;

void start_op(async_operation<int>* op)
{
	print(PRI1, "start_op()\n");

	operation = [op](int i)
	{
		print(PRI1, "start_op()\n");

		async_operation<int>* om_async_operation_t = op;

		if (om_async_operation_t)
		{
			print(PRI1, "start_op(): om_async_operation_t->set_result(%d)\n", i);
			om_async_operation_t->set_result(i);
			om_async_operation_t->completed();
		}
		else
		{
			// This can occur when the async_operation_base has gone out of scope.
			print(PRI1, "start_op() : Warning: om_async_operation_t == nullptr\n");
		}
	};

	std::thread thread1([]() {
		print(PRI1, "start_op(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		print(PRI1, "start_op(): thread1: before operation(10);\n");
		operation(10);
		print(PRI1, "start_op(): thread1: after operation(10);\n");
		});
	thread1.detach();
}

async_task<int> coroutine5()
{
	print(PRI1, "coroutine5()\n");
	print(PRI1, "coroutine5(): co_await op;\n");
	async_operation<int> op;
	start_op(&op);
	int v = co_await op;
	print(PRI1, "coroutine5(): co_return %d;\n", v+1);
	co_return v+1;
}

async_task<int> coroutine4()
{
	print(PRI1, "coroutine4(): async_task<int> a = coroutine5();\n");
	async_task<int> a = coroutine5();
	print(PRI1, "coroutine4(): int v = co_await a;\n");
	int v = co_await a;
	print(PRI1, "coroutine4(): co_return %d;\n", v+1);
	co_return v+1;
}

async_task<int> coroutine3()
{
	print(PRI1, "coroutine3(): async_task<int> a1 = coroutine4();\n");
	async_task<int> a1 = coroutine4();
	print(PRI1, "coroutine3(): int v = co_await a1;\n");
	int v1 = co_await a1;

	print(); print(PRI1, "coroutine3(): async_task<int> a2 = coroutine4();\n");
	async_task<int> a2 = coroutine4();
	print(PRI1, "coroutine3(): int v = co_await a2;\n");
	int v2 = co_await a2;

	fprintf(stderr, "\n");
	print(PRI1, "coroutine3(): co_return %d;\n", v1 + v2 + 1);
	co_return v1+v2+1;
}

async_task<int> coroutine2()
{
	print(PRI1, "coroutine2(): async_task<int> a = coroutine3();\n");
	async_task<int> a = coroutine3();
	print(PRI1, "coroutine2(): int v = co_await a;\n");
	int v = co_await a;
	print(PRI1, "coroutine2(): co_return %d;\n", v+1);
	co_return v+1;
}

async_task<int> coroutine1()
{
	print(PRI1, "coroutine1(): async_task<int> a = coroutine2();\n");
	async_task<int> a = coroutine2();
	print(PRI1, "coroutine1(): int v = co_await a;\n");
	int v = co_await a;
	print(PRI1, "coroutine1(): co_return %d;\n", v+1);
	co_return v+1;
}

int main()
{
	print(PRI1, "main(): async_task<int> a = coroutine1();\n");
	async_task<int> a = coroutine1();

	print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	print(PRI1, "main(): int v = a.get();\n");
	int v = a.get();
	print(PRI1, "main(): v = %d\n", v);

	print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	print(PRI1, "main(): return 0;\n");
	return 0;
}
