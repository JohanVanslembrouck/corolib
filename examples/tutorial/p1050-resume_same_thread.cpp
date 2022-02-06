/** 
 *  Filename: p1050-resume_same_thread.cpp
 *  Description:
 *
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 *
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

struct resume_same_thread
{
	bool await_ready() noexcept
	{
		print(PRI1, "resume_same_thread::await_ready()\n");
		return true;
	}

	void await_suspend(std::experimental::coroutine_handle<> handle) noexcept
	{
		print(PRI1, "resume_same_thread::await_suspend(...);\n");
	}

	void await_resume() noexcept
	{
		print(PRI1, "resume_same_thread::await_resume()\n");
	}
};

async_task<int> coroutine5() {
	print(PRI1, "coroutine5(): co_await resume_same_thread\n");
	co_await resume_same_thread();
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
	print(PRI1, "coroutine3(): int v = co_await a1;\n");
	int v1 = co_await a1;

	print();
	print(PRI1, "coroutine3(): async_task<int> a2 = coroutine4();\n");
	async_task<int> a2 = coroutine4();
	print(PRI1, "coroutine3(): int v = co_await a2;\n");
	int v2 = co_await a2;

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
	print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v+1);
	co_return v+1;
}

/**
 * Because main() cannot be a coroutine (it cannot return a coroutine type),
 * it cannot use co_await. Instead it calls get_result() on the coroutine object
 * returned from coroutine1().
 */
int main()
{
	print(PRI1, "main(): async_task<int> a = coroutine1();\n");
	async_task<int> a = coroutine1();
	print(PRI1, "main(): int v = a.get_result();\n");
	int v = a.get_result();
	print(PRI1, "main(): v = %d\n", v);
	print(PRI1, "main(): return 0;\n");
	return 0;
}
