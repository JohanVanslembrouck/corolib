
/**
 * @file p1912-async_queue-fibonacci.cpp
 * @brief This example shows how to generate the Fibonacci sequence without C++ generators (co_yield)
 * but with an async_queue containing 1 element.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/async_task.h>
#include <corolib/when_all.h>
#include <corolib/async_queue.h>
#include <corolib/print.h>

using namespace corolib;

const int QUEUESIZE = 1;
async_queue<int, QUEUESIZE> q;

const int NR_OPERATIONS = 20;

async_task<void> fibonacciGenerator()
{
    print(PRI1, "fibonacciGenerator(): entered\n");

    int value1 = 0;
    int value2 = 1;
    for (int i = 0; i < NR_OPERATIONS; i++)
    {
        print(PRI1, "fibonacciGenerator(): pushing value %d\n", value1);
        co_await q.push(value1);
        int temp = value1 + value2;
        value1 = value2;
        value2 = temp;
    }

    print(PRI1, "fibonacciGenerator(): co_return;\n");
    co_return;
}

async_task<void> fibonacciConsumer()
{
    print(PRI1, "fibonacciConsumer(): entered\n");

    for (int i = 0; i < NR_OPERATIONS; i++)
    {
        int val = co_await q.pop();
        print(PRI1, "fibonacciConsumer(): popped value  %d\n", val);
        printf("Fibonacci(%d) = %d\n", i, val);
    }

    print(PRI1, "fibonacciConsumer(): co_return;\n");
    co_return;
}

async_task<void> start1()
{
    print(PRI1, "start1(): async_task<void> p = fibonacciGenerator();\n");
    async_task<void> p = fibonacciGenerator();

    print(PRI1, "start1(): async_task<void> c = fibonacciConsumer()\n");
    async_task<void> c = fibonacciConsumer();

    print(PRI1, "start1(): co_await when_all...\n");
#if 0
	// The following statement does not compile with g++ 11.3.0
    co_await when_all({ &p, &c });
#else
    co_await when_all(p, c);
#endif
    print(PRI1, "start1(); co_return;\n");
    co_return;
}

async_task<void> start2()
{
    print(PRI1, "start2(): async_task<void> c = fibonacciConsumer()\n");
    async_task<void> c = fibonacciGenerator();

    print(PRI1, "start2(): async_task<void> p = fibonacciGenerator();\n");
    async_task<void> p = fibonacciConsumer();

    print(PRI1, "start2(): co_await when_all...\n");
#if 0
	// The following statement does not compile with g++ 11.3.0
    co_await when_all({ &p, &c });
#else
    co_await when_all(p, c);
#endif
    print(PRI1, "start2(); co_return;\n");
    co_return;
}

int main()
{
    set_priority(0x01);
 
    print(PRI1, "main(): async_task<void> t1 = start1();\n");
    async_task<void> t1 = start1();
    print(PRI1, "main(): t1.wait();\n");
    t1.wait();

    printf("\n");
 
    print(PRI1, "main(): async_task<void> t2 = start2();\n");
    async_task<void> t2 = start2();
    print(PRI1, "main(): t2.wait();\n");
    t2.wait();

    print(PRI1, "main(): return 0;\n");
    return 0;
}
