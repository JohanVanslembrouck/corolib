/**
 * @file p2206-async_operation-immediate.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <algorithm>

#include <corolib/when_all.h>

#include "p2200.h"

async_task<bool> sortRandumNumberVector(int size)
{
    // set up random number generation 
    std::random_device rd;
    std::default_random_engine engine{ rd() };
    std::uniform_int_distribution ints;

    print(PRI1, "sortRandumNumberVector(): creating vector of random ints\n");
    std::vector<int> values(size);
    std::ranges::generate(values, [&]() {return ints(engine); });

    Sorter sorter(UseMode::USE_IMMEDIATE_COMPLETION);

#if !USE_LAZY_START_OPS
    print(PRI1, "sortRandumNumberVector(): starting sortCoroutine\n");
    async_task<void> result = sortCoroutine(sorter, values);
#else
    print(PRI1, "sortRandumNumberVector(): starting sortCoroutine_lso\n");
    async_task<void> result = sortCoroutine_lso(sorter, values);
#endif

    co_await result;

    print(PRI1, "sortRandumNumberVector(): confirming that vector is sorted\n");
    bool sorted = std::ranges::is_sorted(values);
    print(PRI1, "sortRandumNumberVector(): values is %s sorted\n", sorted ? "" : " not");

    co_return sorted;
}

async_task<bool> sort3RandumNumberVectors()
{
    print(PRI1, "sort3RandumNumberVectors(): async_task<bool> t1 = sortRandumNumberVector(9'000'000);\n");
    async_task<bool> t1 = sortRandumNumberVector(9'000'000);
    print(PRI1, "sort3RandumNumberVectors(): async_task<bool> t2 = sortRandumNumberVector(10'000'000);\n");
    async_task<bool> t2 = sortRandumNumberVector(10'000'000);
    print(PRI1, "sort3RandumNumberVectors(): async_task<bool> t3 = sortRandumNumberVector(11'000'000);\n");
    async_task<bool> t3 = sortRandumNumberVector(11'000'000);

    print(PRI1, "sort3RandumNumberVectors(): co_await when_all(t1, t2, t3);\n");
    co_await when_all(t1, t2, t3);

    bool res = t1.get_result() & t2.get_result() & t3.get_result();
    print(PRI1, "sort3RandumNumberVectors(): res = %d\n", res);
    co_return res;
}

int main() 
{
   set_priority(0x01);        // Use 0x03 to follow the flow in corolib

   for (int i = 1; i <= 3; ++i)
   {
       print(PRI1, "main(): async_task<bool> t = sortRandumNumberVector(i * 10'000'000);\n", i);
       async_task<bool> t = sortRandumNumberVector(i * 10'000'000);
       print(PRI1, "main(): bool res = t.get_result();\n");
       bool res = t.get_result();
       print(PRI1, "main(): res = %d\n", res);
   }

   print(PRI1, "main(): async_task<bool> t = sort3RandumNumberVectors()\n");
   async_task<bool> t = sort3RandumNumberVectors();
   print(PRI1, "main(): bool res = t.get_result();\n");
   bool res = t.get_result();
   print(PRI1, "main(): res = %d\n", res);

   return 0;
}
