/**
 * @file p2210-no-coroutines.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <chrono>
#include <algorithm>

#include "p2220-sort.h"

#include "corolib/print.h"

using namespace corolib;

bool sortRandomNumberVector(int num_elem, bool concurrent)
{
    std::srand(0);
    std::vector<int> v;

    v.reserve(num_elem);
    for (int i = num_elem - 1; i >= 0; i--)
        v.push_back(rand());

    auto t0 = std::chrono::high_resolution_clock::now();

    if (concurrent)
		concurrent_sort(v);
    else
        serial_sort(v);

    auto t1 = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

    bool is_sorted = std::is_sorted(v.begin(), v.end());
    if (is_sorted)
        print(PRI1, "Sorted\n");
    else
        print(PRI1, "Not sorted\n");

    print(PRI1, "Took %dms\n", int(dt));
    return is_sorted;
}

int main()
{
    for (int i = 1; i <= 3; ++i)
    {
        print(PRI1, "main(): bool res = sortRandomNumberVectorSerial(%d * 10'000'000, concurrent = false);\n", i);
        bool res = sortRandomNumberVector(i * 10'000'000, false);
        print(PRI1, "main(): res = %d\n", res);
    }
    for (int i = 1; i <= 3; ++i)
    {
        print(PRI1, "main(): bool res = sortRandomNumberVectorSerial(%d * 10'000'000, concurrent = true);\n", i);
        bool res = sortRandomNumberVector(i * 10'000'000, true);
        print(PRI1, "main(): res = %d\n", res);
    }

    return 0;
}
