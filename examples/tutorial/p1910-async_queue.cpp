
/**
 * @file p1910-async_queue.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/async_task.h>
#include <corolib/when_all.h>
#include <corolib/async_queue.h>
#include <corolib/print.h>

using namespace corolib;

const int QUEUESIZE = 16;       // Also works with QUEUESIZE = 1
async_queue<int, QUEUESIZE> q;

const int NR_OPERATIONS = 100;
const int MULTIPLIER = 5;

/**
 * @brief producer() pushes items to the queue, possibly until the queue is full (consumer() not pop-ing).
 * When the queue is full, producer() suspends itself.
 * When producer() notices that consumer() is waiting to pop (queue was empty), 
 * it will resume consumer() after it has pushed an item onto the queue.
 */
async_task<void> producer()
{
    print(PRI1, "producer(): entered\n");
    for (int i = 0; i < NR_OPERATIONS; i++)
    {
        print(PRI1, "producer(): pushing value %d\n", MULTIPLIER * i);
        co_await q.push(MULTIPLIER * i);
    }
    print(PRI1, "producer(): co_return;\n");
    co_return;
}

/**
 * @brief consumer() removes items from the queue, possibly until the queue is empty (poducer() not pushing).
 * When the queue is empty, consumer() suspends itself.
 * When consumer() notices that producer() is waiting to push (queue was full), 
 * it will resume producer() after it has popped an element from the queue.
 */
async_task<void> consumer()
{
    int val = -1;
    int prev_val = -MULTIPLIER;
    int nrErrors = 0;

    print(PRI1, "consumer(): entered\n");
    for (int i = 0; i < NR_OPERATIONS; i++)
    {
        val = co_await q.pop();
        if (val != (prev_val + MULTIPLIER))
        {
            print(PRI1, "consumer(): prev_val = %d, val = %d\n", prev_val, val);
            nrErrors++;
        }
        prev_val = val;
        print(PRI1, "consumer(): popped value  %d\n", val);
    }
    print(PRI1, "consumer(): nrErrors = %d\n", nrErrors);
    print(PRI1, "consumer(): co_return;\n");
    co_return;
}

async_task<void> start1()
{
    print(PRI1, "start1(): async_task<void> p = producer();\n");
    async_task<void> p = producer();

    print(PRI1, "start1(): async_task<void> c = consumer()\n");
    async_task<void> c = consumer();

    print(PRI1, "start1(): co_await when_all...\n");
#if 0
	// The following statement does not compile with g++ 11.3.0
    co_await when_all<async_task<void>>({ &p, &c });
#else
	// Split it into two lines:
	when_all<async_task<void>> wa({ &p, &c });
	co_await wa;
#endif
    print(PRI1, "start1(); co_return;\n");
    co_return;
}

async_task<void> start2()
{
    print(PRI1, "start2(): async_task<void> c = consumer()\n");
    async_task<void> c = consumer();

    print(PRI1, "start2(): async_task<void> p = producer();\n");
    async_task<void> p = producer();

    print(PRI1, "start2(): co_await when_all...\n");
#if 0
	// The following statement does not compile with g++ 11.3.0
    co_await when_all<async_task<void>>({ &p, &c });
#else
	// Split it into two lines:
	when_all<async_task<void>> wa({ &p, &c });
    co_await wa;
#endif
    print(PRI1, "start2(); co_return;\n");
    co_return;
}

int main()
{
    set_priority(0x01);        // Use 0x03 to follow the flow in corolib and async_queue.h
 
    print(PRI1, "main(): async_task<void> t1 = start1();\n");
    async_task<void> t1 = start1();
    print(PRI1, "main(): t1.wait();\n");
    t1.wait();
 
    print(PRI1, "main(): async_task<void> t2 = start2();\n");
    async_task<void> t2 = start2();
    print(PRI1, "main(): t2.wait();\n");
    t2.wait();

    print(PRI1, "main(): return 0;\n");
    return 0;
}
