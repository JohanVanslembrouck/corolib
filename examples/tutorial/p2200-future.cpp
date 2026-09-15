/**
 * @file p2200-future.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <random>
#include <string>
#include <thread>
#include <vector>
#include <future>
#include <algorithm>

template<typename TYPE>
class async_operation
{
public:
    async_operation()
    {
        printf("async_operation::async_operation(...)\n");
    }

    void get_result() {
        printf("async_operation::get_result()\n");
        m_fut.get();
    }
    std::future<TYPE> m_fut;
};

void start_sorting_impl(async_operation<void>& op, const std::vector<int>::iterator& begin, 
                                                   const std::vector<int>::iterator& end) noexcept
{
    printf("sort_operation::start_sorting_impl(): begin\n");
    op.m_fut = std::async(std::launch::async,
        [&begin, &end] {
            printf("sort_operation::start_sorting_impl(): begin sorting\n");
            std::sort(begin, end);
            printf("sort_operation::start_sorting_impl(): end sorting\n");
        });
    printf("sort_operation::start_sorting_impl(): end\n");
}

async_operation<void> start_sorting(const std::vector<int>::iterator& begin, const std::vector<int>::iterator& end)
{
    async_operation<void> ret;
    start_sorting_impl(ret, begin, end);
    return ret;
}

#if 1

void sortVector(std::vector<int>& values)
{
    printf("sortVector: start\n");

    size_t middle{ values.size() / 2 }; // middle element index

    std::vector<int>::iterator b = values.begin();
    std::vector<int>::iterator m = values.begin() + middle;
    std::vector<int>::iterator e = values.end();
    async_operation<void> op1 = start_sorting(b, m);
    async_operation<void> op2 = start_sorting(m, e);
    op1.get_result();
    op2.get_result();

    // merge the two sorted sub-vectors
    printf("sortVector: merging results\n");
    std::inplace_merge(b, m, e);
}

#else

/*
// Deadlock on Ubuntu?
./p2200-future
main(): bool res = sortRandomNumberVector(1 * 10'000'000);
sortRandomNumberVector(): creating vector of random ints
sortVector: start
async_operation::async_operation(...)
sort_operation::start_sorting_impl(): begin
sort_operation::start_sorting_impl(): end
async_operation::async_operation(...)
sort_operation::start_sorting_impl(): begin sorting
sort_operation::start_sorting_impl(): begin
sort_operation::start_sorting_impl(): end
async_operation::get_result()
sort_operation::start_sorting_impl(): begin sorting
^C
*/

void sortVector(std::vector<int>& values)
{
    printf("sortVector: start\n");

    size_t middle{ values.size() / 2 }; // middle element index

    async_operation<void> op1 = start_sorting(values.begin(), values.begin() + middle);
    async_operation<void> op2 = start_sorting(values.begin() + middle, values.end());
    op1.get_result();
    op2.get_result();

    // merge the two sorted sub-vectors
    printf("sortVector: merging results\n");
    std::inplace_merge(values.begin(), values.begin() + middle, values.end());
}

#endif

bool sortRandomNumberVector(int size)
{
    // set up random number generation 
    std::random_device rd;
    std::default_random_engine engine{ rd() };
    std::uniform_int_distribution ints;

    printf("sortRandomNumberVector(): creating vector of random ints\n");
    std::vector<int> values(size);
    std::ranges::generate(values, [&]() {return ints(engine); });

    sortVector(values);

    printf("sortRandomNumberVector(): confirming that vector is sorted\n");
    bool sorted = std::ranges::is_sorted(values);
    printf("sortRandomNumberVector(): values is %s sorted\n", sorted ? "" : " not");

    return sorted;
}

bool sort3RandomNumberVectors()
{
    printf("sort3RandomNumberVectors(): bool res1 = sortRandomNumberVector(9'000'000);\n");
    bool res1 = sortRandomNumberVector(9'000'000);
    printf("sort3RandomNumberVectors(): bool res2 = sortRandomNumberVector(10'000'000);\n");
    bool res2 = sortRandomNumberVector(10'000'000);
    printf("sort3RandomNumberVectors(): bool res3 = sortRandomNumberVector(11'000'000);\n");
    bool res3 = sortRandomNumberVector(11'000'000);

    bool res = res1 && res2 && res3;
    printf("sort3RandomNumberVectors(): res = %d\n", res);
    return res;
}

int main()
{
    for (int i = 1; i <= 3; ++i)
    {
        printf("main(): bool res = sortRandomNumberVector(%d * 10'000'000);\n", i);
        bool res = sortRandomNumberVector(i * 10'000'000);
        printf("main(): res = %d\n", res);
    }

    printf("main(): bool res = sort3RandomNumberVectors()\n");
    bool res = sort3RandomNumberVectors();
    printf("main(): bool res = %d\n", res);

    return 0;
}
