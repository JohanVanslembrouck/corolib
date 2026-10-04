/**
 * @file p2210-sort.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <thread>
#include <algorithm>

#include <corolib/print.h>

#include "p2210-sort.h"

using namespace corolib;

// Concurrent sort
void sortVector(std::vector<int>& values)
{
    print(PRI1, "sortVector: start\n");

    size_t middle{ values.size() / 2 }; // middle element index

    std::thread thread1([&values, middle]()
        { std::sort(values.begin(), values.begin() + middle); });
    std::thread thread2([&values, middle]()
        { std::sort(values.begin() + middle, values.end()); });
    thread1.join();
    thread2.join();

    // merge the two sorted sub-vectors
    print(PRI1, "sortVector: merging results\n");
    std::inplace_merge(values.begin(), values.begin() + middle, values.end());

    print(PRI1, "sortVector: return;\n");
}

