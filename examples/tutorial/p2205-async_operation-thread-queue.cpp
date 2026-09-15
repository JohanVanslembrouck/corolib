/**
 * @file p2205-async_operation-thread-queue.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <algorithm>

#include <corolib/when_all.h>

#include "p2200.h"

EventQueueThrFunctionVoidVoid eventQueueThr;

async_task<bool> sortRandomNumberVector(int size)
{
    // set up random number generation 
    std::random_device rd;
    std::default_random_engine engine{ rd() };
    std::uniform_int_distribution ints;

    print(PRI1, "sortRandomNumberVector(): creating vector of random ints\n");
    std::vector<int> values(size);
    std::ranges::generate(values, [&]() {return ints(engine); });

    Sorter sorter(UseMode::USE_THREAD_QUEUE, nullptr, &eventQueueThr);

#if !USE_LAZY_START_OPS
    print(PRI1, "sortRandomNumberVector(): starting sortVector\n");
    async_task<void> result = sortVector(sorter, values);
#else
    print(PRI1, "sortRandomNumberVector(): starting sortVector_lso\n");
    async_task<void> result = sortVector_lso(sorter, values);
#endif

    co_await result;

    print(PRI1, "sortRandomNumberVector(): confirming that vector is sorted\n");
    bool sorted = std::ranges::is_sorted(values);
    print(PRI1, "sortRandomNumberVector(): values is %s sorted\n", sorted ? "" : " not");

    co_return sorted;
}

async_task<bool> sort3RandomNumberVectors()
{
    print(PRI1, "sort3RandomNumberVectors(): async_task<bool> t1 = sortRandomNumberVector(9'000'000);\n");
    async_task<bool> t1 = sortRandomNumberVector(9'000'000);
    print(PRI1, "sort3RandomNumberVectors(): async_task<bool> t2 = sortRandomNumberVector(10'000'000);\n");
    async_task<bool> t2 = sortRandomNumberVector(10'000'000);
    print(PRI1, "sort3RandomNumberVectors(): async_task<bool> t3 = sortRandomNumberVector(11'000'000);\n");
    async_task<bool> t3 = sortRandomNumberVector(11'000'000);

    print(PRI1, "sort3RandomNumberVectors(): co_await when_all(t1, t2, t3);\n");
    co_await when_all(t1, t2, t3);

    bool res = t1.get_result() & t2.get_result() & t3.get_result();
    print(PRI1, "sort3RandomNumberVectors(): res = %d\n", res);
    co_return res;
}

int main()
{
    set_priority(0x01);        // Use 0x03 to follow the flow in corolib

    for (int i = 1; i <= 3; ++i)
    {
        print(PRI1, "main(): async_task<bool> t = sortRandomNumberVector(%d * 10'000'000);\n", i);
        async_task<bool> t = sortRandomNumberVector(i * 10'000'000);
        print(PRI1, "main(): runEventQueueThr(eventQueueThr)\n");
        runEventQueueThr(eventQueueThr);
        print(PRI1, "main(): bool res = t.get_result();\n");
        bool res = t.get_result();
        print(PRI1, "main(): res = %d\n", res);
    }

    print(PRI1, "main(): async_task<bool> t = sort3RandomNumberVectors()\n");
    async_task<bool> t = sort3RandomNumberVectors();
    print(PRI1, "main(): runEventQueueThr(eventQueue)\n");
    runEventQueueThr(eventQueueThr);
    print(PRI1, "main(): bool res = t.get_result();\n");
    bool res = t.get_result();
    print(PRI1, "main(): res = %d\n", res);

    print(PRI1, "main(): return 0;\n");
    return 0;
}