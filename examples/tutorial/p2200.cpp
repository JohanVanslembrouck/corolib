/**
 * @file p2200.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <corolib/when_all.h>

#include "p2200.h"

async_operation<void> Sorter::start_sorting(auto begin, auto end)
{
    int index = get_free_index();
    print(PRI1, "Sorter::start_sorting(): index = %d\n", index);
    async_operation<void> ret{ this, index, false };
    start_sorting_impl(index, begin, end);
    print(PRI1, "Sorter::start_sorting(): return ret;\n");
    return ret;
}

void Sorter::start_sorting_impl(int idx, auto begin, auto end)
{
    print(PRI1, "Sorter::start_sorting_impl(): idx = %d\n", idx);

    switch (m_useMode)
    {
    case UseMode::USE_NONE:
        print(PRI1, "Sorter::start_sorting_impl(): UseMode::USE_NONE\n");
        print(PRI1, "Sorter::start_sorting_impl(): begin sorting\n");
        std::sort(begin, end);
        print(PRI1, "Sorter::start_sorting_impl(): end sorting\n");
        break;
    case UseMode::USE_EVENTQUEUE:
    {
        print(PRI1, "Sorter::start_sorting_impl(): UseMode::USE_EVENTQUEUE\n");
        print(PRI1, "Sorter::start_sorting_impl(): begin sorting\n");
        std::sort(begin, end);
        print(PRI1, "Sorter::start_sorting_impl(): end sorting\n");
        if (m_eventQueue)
            m_eventQueue->push([this, idx]() { completionHandler_v(idx); });
        break;
    }
    case UseMode::USE_THREAD:
    {
        print(PRI1, "Sorter::start_sorting_impl(): UseMode::USE_THREAD\n");
// #if USE_THREAD_POOL  // do not use USE_THREAD_POOL: application crashes: FFS
#if 0
        m_pool.enqueue(
            [this, idx, begin, end]() {
                print(PRI1, "Sorter::start_sorting_impl(): thread1: begin sorting\n");
                std::sort(begin, end);
                print(PRI1, "Sorter::start_sorting_impl(): thread1: end sorting\n");

                print(PRI1, "Sorter::start_sorting_impl(): thread1: completionHandler_v(idx = %d)\n", idx);
                completionHandler_v(idx);
                print(PRI1, "Sorter::start_sorting_impl(): thread1: return\n");
            });
#else
        std::thread thread1(
            [this, idx, begin, end]() {
                print(PRI1, "Sorter::start_sorting_impl(): thread1: begin sorting\n");
                std::sort(begin, end);
                print(PRI1, "Sorter::start_sorting_impl(): thread1: end sorting\n");

                print(PRI1, "Sorter::start_sorting_impl(): thread1: completionHandler_v(idx = %d)\n", idx);
                completionHandler_v(idx);
                print(PRI1, "Sorter::start_sorting_impl(): thread1: return\n");
            });
        thread1.detach();
#endif
        break;
    }
    case UseMode::USE_THREAD_QUEUE:
    {
        print(PRI1, "Sorter::start_sorting_impl(): UseMode::USE_THREAD_QUEUE\n");
        m_queueSize++;
#if USE_THREAD_POOL
        m_pool.enqueue(
            [this, idx, begin, end]() {
                print(PRI1, "Sorter::start_sorting_impl(): thread1: begin sorting\n");
                std::sort(begin, end);
                print(PRI1, "Sorter::start_sorting_impl(): thread1: end sorting\n");
                if (m_eventQueueThr)
                    m_eventQueueThr->push([this, idx]() { completionHandler_v(idx); });
                print(PRI1, "Sorter::start_sorting_impl(): thread1: return;\n");
            }
        );
#else
        std::thread thread1(
            [this, idx, begin, end]() {
                print(PRI1, "Sorter::start_sorting_impl(): thread1: begin sorting\n");
                std::sort(begin, end);
                print(PRI1, "Sorter::start_sorting_impl(): thread1: end sorting\n");
                if (m_eventQueueThr)
                    m_eventQueueThr->push([this, idx]() { completionHandler_v(idx); });
                print(PRI1, "Sorter::start_sorting_impl(): thread1: return;\n");
            });
        thread1.detach();
#endif
        break;
    }
    case UseMode::USE_IMMEDIATE_COMPLETION:
        print(PRI1, "Sorter::start_sorting_impl(): UseMode::USE_IMMEDIATE_COMPLETION\n");
        print(PRI1, "Sorter::start_sorting_impl(): begin sorting\n");
        std::sort(begin, end);
        print(PRI1, "Sorter::start_sorting_impl(): end sorting\n");
        print(PRI1, "Sorter::try_start(): before completionHandler_v(idx);\n");
        completionHandler_v(idx);
        print(PRI1, "Sorter::try_start(): after completionHandler_v(idx);\n");
        break;
    }

    print(PRI1, "Sorter::start_sort(): return\n");
}

// -----------------------------------------------------------------------------

Sorter::sort_operation Sorter::start_sorting(Sorter* sorter, std::vector<int>::iterator& begin, std::vector<int>::iterator& end)
{
    return sort_operation(sorter, begin, end);
}

// -----------------------------------------------------------------------------

Sorter::sort_operation_impl::sort_operation_impl(Sorter* sorter, std::vector<int>::iterator& begin, std::vector<int>::iterator& end)
    : m_sorter(sorter)
    , m_begin(begin)
    , m_end(end)
{
    print(PRI1, "sort_operation_impl::sort_operation_impl()\n");
}

bool Sorter::sort_operation_impl::try_start(async_operation_ls_base& operation) noexcept
{
    print(PRI1, "sort_operation_impl::try_start()\n");

    switch (m_sorter->m_useMode)
    {
    case UseMode::USE_NONE:
        print(PRI1, "sort_operation_impl::try_start(): UseMode::USE_NONE\n");
        print(PRI1, "sort_operation_impl::try_start(): begin sorting\n");
        std::sort(m_begin, m_end);
        print(PRI1, "sort_operation_impl::try_start(): end sorting\n");
        break;
    case UseMode::USE_EVENTQUEUE:
    {
        print(PRI1, "sort_operation_impl::try_start(): UseMode::USE_EVENTQUEUE\n");
        print(PRI1, "sort_operation_impl::try_start(): begin sorting\n");
        std::sort(m_begin, m_end);
        print(PRI1, "sort_operation_impl::try_start(): end sorting\n");
        if (m_sorter->m_eventQueue)
            m_sorter->m_eventQueue->push([this, &operation]() { operation.completed(); });
        break;
    }
    case UseMode::USE_THREAD:
    {
        print(PRI1, "sort_operation_impl::try_start(): UseMode::USE_THREAD\n");
// #if USE_THREAD_POOL  // do not use USE_THREAD_POOL: application crashes: FFS
#if 0
        m_sorter->m_pool.enqueue(
            [this, &operation]() {
                print(PRI1, "sort_operation_impl::try_start(): thread1: begin sorting\n");
                std::sort(m_begin, m_end);
                print(PRI1, "sort_operation_impl::try_start(): thread1: end sorting\n");

                print(PRI1, "sort_operation_impl::try_start(): thread1: operation.completed()\n");
                operation.completed();
                print(PRI1, "sort_operation_impl::try_start(): thread1: return\n");
            });
#else
        std::thread thread1(
            [this, &operation]() {
                print(PRI1, "sort_operation_impl::try_start(): thread1: begin sorting\n");
                std::sort(m_begin, m_end);
                print(PRI1, "sort_operation_impl::try_start(): thread1: end sorting\n");

                print(PRI1, "sort_operation_impl::try_start(): thread1: operation.completed()\n");
                operation.completed();
                print(PRI1, "sort_operation_impl::try_start(): thread1: return\n");
            });
        thread1.detach();
#endif
        break;
    }
    case UseMode::USE_THREAD_QUEUE:
    {
        print(PRI1, "sort_operation_impl::try_start(): UseMode::USE_THREAD_QUEUE\n");
        m_sorter->m_queueSize++;
#if USE_THREAD_POOL
        m_sorter->m_pool.enqueue(
            [this, &operation]() {
                print(PRI1, "sort_operation_impl::try_start(): thread1: begin sorting\n");
                std::sort(m_begin, m_end);
                print(PRI1, "sort_operation_impl::try_start(): thread1: end sorting\n");
                if (m_sorter->m_eventQueueThr)
                    m_sorter->m_eventQueueThr->push([this, &operation]() { operation.completed(); });
                print(PRI1, "sort_operation_impl::try_start(): thread1: return;\n");
            });
#else
        std::thread thread1(
            [this, &operation]() {
                print(PRI1, "sort_operation_impl::try_start(): thread1: begin sorting\n");
                std::sort(m_begin, m_end);
                print(PRI1, "sort_operation_impl::try_start(): thread1: end sorting\n");
                if (m_sorter->m_eventQueueThr)
                    m_sorter->m_eventQueueThr->push([this, &operation]() { operation.completed(); });
                print(PRI1, "sort_operation_impl::try_start(): thread1: return;\n");
            });
        thread1.detach();
#endif
        break;
    }
    case UseMode::USE_IMMEDIATE_COMPLETION:
        print(PRI1, "sort_operation_impl::try_start(): UseMode::USE_IMMEDIATE_COMPLETION\n");
        print(PRI1, "sort_operation_impl::try_start(): begin sorting\n");
        std::sort(m_begin, m_end);
        print(PRI1, "sort_operation_impl::try_start(): end sorting\n");
        print(PRI1, "sort_operation_impl::try_start(): before operation.completed()\n");
        operation.completed();
        print(PRI1, "sort_operation_impl::try_start(): after operation.completed()\n");
        break;
    }

    print(PRI1, "sort_operation_impl::try_start(): return\n");
    return true;
}

void Sorter::sort_operation_impl::get_result(async_operation_ls_base&)
{
    print(PRI1, "sort_operation_impl::get_result()\n");
}

// -----------------------------------------------------------------------------

#if 1

async_task<void> sortCoroutine(Sorter& sorter, std::vector<int>& values)
{
   print(PRI1, "sortCoroutine: start\n");

   size_t middle{ values.size() / 2 }; // middle element index

   std::vector<int>::iterator b = values.begin();
   std::vector<int>::iterator m = values.begin() + middle;
   std::vector<int>::iterator e = values.end();
   async_operation<void> op1 = sorter.start_sorting(b, m);
   async_operation<void> op2 = sorter.start_sorting(m, e);
   print(PRI1, "sortCoroutine: co_await when_all(op1, op2);\n");
   co_await when_all(op1, op2);

   // merge the two sorted sub-vectors
   print(PRI1, "sortCoroutine: merging results\n");
   std::inplace_merge(b, m, e);

   print(PRI1, "sortCoroutine: co_return\n");
   co_return;
}

#else

async_task<void> sortCoroutine(Sorter& sorter, std::vector<int>& values)
{
    print(PRI1, "sortCoroutine: start\n");

    size_t middle{ values.size() / 2 }; // middle element index

    async_operation<void> op1 = sorter.start_sorting(values.begin(), values.begin() + middle);
    async_operation<void> op2 = sorter.start_sorting(values.begin() + middle, values.end());
    print(PRI1, "sortCoroutine: co_await when_all(op1, op2);\n");
    co_await when_all(op1, op2);

    // merge the two sorted sub-vectors
    print(PRI1, "sortCoroutine: merging results\n");
    std::inplace_merge(values.begin(), values.begin() + middle, values.end());

    print(PRI1, "sortCoroutine: co_return\n");
    co_return;
}

#endif

async_task<void> sortCoroutine_lso(Sorter& sorter, std::vector<int>& values)
{
    print(PRI1, "sortCoroutine_lso: start\n");

    size_t middle{ values.size() / 2 }; // middle element index

    // gcc: define iterators explicitly, otherwise they are not passed correctly to sort_operation_impl: application crashes
    std::vector<int>::iterator b = values.begin();
    std::vector<int>::iterator m = values.begin() + middle;
    std::vector<int>::iterator e = values.end();
    Sorter::sort_operation op1 = sorter.start_sorting(&sorter, b, m);
    Sorter::sort_operation op2 = sorter.start_sorting(&sorter, m, e);

    print(PRI1, "sortCoroutine_lso: co_await when_all(op1, op2);\n");
    co_await when_all(op1, op2);

    // merge the two sorted sub-vectors
    print(PRI1, "sortCoroutine_lso: merging results\n");
    std::inplace_merge(b, m, e);

    print(PRI1, "sortCoroutine_lso: co_return\n");
    co_return;
}
