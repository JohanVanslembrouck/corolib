/**
 * @file p2200-sort.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <algorithm>
#include <random>

#include <corolib/print.h>
#include <corolib/when_all.h>

#include "p2200-sort.h"

// Serial sort: not using coroutines
void sortVector(std::vector<int>& values)
{
    print(PRI1, "sortVector: start\n");

    size_t middle{ values.size() / 2 }; // middle element index

    std::sort(values.begin(), values.begin() + middle);
    std::sort(values.begin() + middle, values.end());

    // merge the two sorted sub-vectors
    print(PRI1, "sortVector: merging results\n");
    std::inplace_merge(values.begin(), values.begin() + middle, values.end());

    print(PRI1, "sortVector: return;\n");
}

// Serial sort: not using coroutines
bool sortRandomNumberVectorSerial(int size)
{
    // set up random number generation 
    std::random_device rd;
    std::default_random_engine engine{ rd() };
    std::uniform_int_distribution ints;

    print(PRI1, "sortRandomNumberVectorSerial(): creating vector of random ints\n");
    std::vector<int> values(size);
    std::ranges::generate(values, [&]() {return ints(engine); });

    auto t0 = std::chrono::high_resolution_clock::now();

    sortVector(values);

    auto t1 = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    print(PRI1, "sortRandomNumberVectorSerial(): confirming that vector is sorted\n");
    bool sorted = std::ranges::is_sorted(values);
    print(PRI1, "sortRandomNumberVectorSerial(): values is %s sorted\n", sorted ? "" : " not");

    print(PRI1, "sortRandomNumberVectorSerial(): took %dms\n", int(dt));
    return sorted;
}

#if 1

async_task<void> sortVector(Sorter& sorter, std::vector<int>& values)
{
    print(PRI1, "sortVector: start\n");

    size_t middle{ values.size() / 2 }; // middle element index

    std::vector<int>::iterator b = values.begin();
    std::vector<int>::iterator m = values.begin() + middle;
    std::vector<int>::iterator e = values.end();
    async_operation<void> op1 = sorter.start_sorting(b, m);
    async_operation<void> op2 = sorter.start_sorting(m, e);
    print(PRI1, "sortVector: co_await when_all(op1, op2);\n");
    co_await when_all(op1, op2);

    // merge the two sorted sub-vectors
    print(PRI1, "sortVector: merging results\n");
    std::inplace_merge(b, m, e);

    print(PRI1, "sortVector: co_return;\n");
    co_return;
}

#else

// This version crashes on Ubuntu:
async_task<void> sortVector(Sorter& sorter, std::vector<int>& values)
{
    print(PRI1, "sortVector: start\n");

    size_t middle{ values.size() / 2 }; // middle element index

    async_operation<void> op1 = sorter.start_sorting(values.begin(), values.begin() + middle);
    async_operation<void> op2 = sorter.start_sorting(values.begin() + middle, values.end());
    print(PRI1, "sortVector: co_await when_all(op1, op2);\n");
    co_await when_all(op1, op2);

    // merge the two sorted sub-vectors
    print(PRI1, "sortVector: merging results\n");
    std::inplace_merge(values.begin(), values.begin() + middle, values.end());

    print(PRI1, "sortVector: co_return;\n");
    co_return;
}

#endif

async_task<void> sortVector_lso(Sorter& sorter, std::vector<int>& values)
{
    print(PRI1, "sortVector_lso: start\n");

    size_t middle{ values.size() / 2 }; // middle element index

    // gcc: define iterators explicitly, otherwise they are not passed correctly to sort_operation_impl: application crashes
    std::vector<int>::iterator b = values.begin();
    std::vector<int>::iterator m = values.begin() + middle;
    std::vector<int>::iterator e = values.end();
    Sorter::sort_operation op1 = sorter.start_sorting(&sorter, b, m);
    Sorter::sort_operation op2 = sorter.start_sorting(&sorter, m, e);

    print(PRI1, "sortVector_lso: co_await when_all(op1, op2);\n");
    co_await when_all(op1, op2);

    // merge the two sorted sub-vectors
    print(PRI1, "sortVector_lso: merging results\n");
    std::inplace_merge(b, m, e);

    print(PRI1, "sortVector_lso: co_return;\n");
    co_return;
}
