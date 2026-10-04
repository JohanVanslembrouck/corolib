/**
 * @file p2210-no-coroutines.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <algorithm>
#include <random>
#include <chrono>

#include <corolib/print.h>

#include "p2210-sort.h"

using namespace corolib;

bool sortRandomNumberVector(int size)
{
    // set up random number generation 
    std::random_device rd;
    std::default_random_engine engine{ rd() };
    std::uniform_int_distribution ints;

    print(PRI1, "sortRandomNumberVector(): creating vector of random ints\n");
    std::vector<int> values(size);
    std::ranges::generate(values, [&]() {return ints(engine); });

    auto t0 = std::chrono::high_resolution_clock::now();

    sortVector(values);
    
    auto t1 = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    print(PRI1, "sortRandomNumberVector(): confirming that vector is sorted\n");
    bool sorted = std::ranges::is_sorted(values);
    print(PRI1, "sortRandomNumberVector(): values is %s sorted\n", sorted ? "" : " not");

    print(PRI1, "sortRandomNumberVector(): took %dms\n", int(dt));

    return sorted;
}

int main()
{
    for (int i = 1; i <= 3; ++i)
    {
        print(PRI1, "main(): bool res = sortRandomNumberVector(%d * 10'000'000);\n", i);
        bool res = sortRandomNumberVector(i * 10'000'000);
        print(PRI1, "main(): res = %d\n", res);
    }

    return 0;
}
