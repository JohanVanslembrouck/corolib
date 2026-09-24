/**
 * @file p2200.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <random>
#include <string>
#include <thread>

#include <corolib/print.h>

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
        print(PRI1, "Sorter::start_sorting_impl(): end\n");
        break;
    }
    case UseMode::USE_THREAD:
    {
        print(PRI1, "Sorter::start_sorting_impl(): UseMode::USE_THREAD\n");
        std::thread thread1(
            [this, idx, begin, end]() {
                print(PRI1, "Sorter::start_sorting_impl(): thread1: begin sorting\n");
                std::sort(begin, end);
                print(PRI1, "Sorter::start_sorting_impl(): thread1: end sorting\n");

                print(PRI1, "Sorter::start_sorting_impl(): thread1: before completionHandler_v(idx = %d)\n", idx);
                completionHandler_v(idx);
                print(PRI1, "Sorter::start_sorting_impl(): thread1: after completionHandler_v(idx = %d)\n", idx);
            });
        thread1.detach();
        break;
    }
    case UseMode::USE_THREAD_QUEUE:
    {
        print(PRI1, "Sorter::start_sorting_impl(): UseMode::USE_THREAD_QUEUE\n");
        if (m_eventQueueThr)
            m_eventQueueThr->incrementPushCounter();

        std::thread thread1(
            [this, idx, begin, end]() {
                print(PRI1, "Sorter::start_sorting_impl(): thread1: begin sorting\n");
                std::sort(begin, end);
                print(PRI1, "Sorter::start_sorting_impl(): thread1: end sorting\n");
                if (m_eventQueueThr)
                    m_eventQueueThr->push([this, idx]() { completionHandler_v(idx); });
                print(PRI1, "Sorter::start_sorting_impl(): thread1: end\n");
            });
        thread1.detach();
        break;
    }
    case UseMode::USE_IMMEDIATE_COMPLETION:
        print(PRI1, "Sorter::start_sorting_impl(): UseMode::USE_IMMEDIATE_COMPLETION\n");
        print(PRI1, "Sorter::start_sorting_impl(): begin sorting\n");
        std::sort(begin, end);
        print(PRI1, "Sorter::start_sorting_impl(): end sorting\n");
        completionHandler_v(idx);
        print(PRI1, "Sorter::try_start(): end\n");
        break;
    }

    print(PRI1, "Sorter::start_sort(): return;\n");
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
        print(PRI1, "sort_operation_impl::try_start(): UseMode::USE_EVENTQUEUE\n");
        print(PRI1, "sort_operation_impl::try_start(): begin sorting\n");
        std::sort(m_begin, m_end);
        print(PRI1, "sort_operation_impl::try_start(): end sorting\n");
        if (m_sorter->m_eventQueue)
            m_sorter->m_eventQueue->push([this, &operation]() { operation.completed(); });
        print(PRI1, "sort_operation_impl::try_start(): end\n");
        break;

    case UseMode::USE_THREAD:
    {
        print(PRI1, "sort_operation_impl::try_start(): UseMode::USE_THREAD\n");
        std::thread thread1(
            [this, &operation]() {
                print(PRI1, "sort_operation_impl::try_start(): thread1: begin sorting\n");
                std::sort(m_begin, m_end);
                print(PRI1, "sort_operation_impl::try_start(): thread1: end sorting\n");

                print(PRI1, "sort_operation_impl::try_start(): thread1: before operation.completed()\n");
                operation.completed();
                print(PRI1, "sort_operation_impl::try_start(): thread1: after operation.completed()\n");
            });
        thread1.detach();
        break;
    }
    case UseMode::USE_THREAD_QUEUE:
    {
        print(PRI1, "sort_operation_impl::try_start(): UseMode::USE_THREAD_QUEUE\n");
        if (m_sorter->m_eventQueueThr)
            m_sorter->m_eventQueueThr->incrementPushCounter();
        std::thread thread1(
            [this, &operation]() {
                print(PRI1, "sort_operation_impl::try_start(): thread1: begin sorting\n");
                std::sort(m_begin, m_end);
                print(PRI1, "sort_operation_impl::try_start(): thread1: end sorting\n");
                if (m_sorter->m_eventQueueThr)
                    m_sorter->m_eventQueueThr->push([this, &operation]() { operation.completed(); });
                print(PRI1, "sort_operation_impl::try_start(): thread1: end\n");
            });
        thread1.detach();
        break;
    }
    case UseMode::USE_IMMEDIATE_COMPLETION:
        print(PRI1, "sort_operation_impl::try_start(): UseMode::USE_IMMEDIATE_COMPLETION\n");
        print(PRI1, "sort_operation_impl::try_start(): begin sorting\n");
        std::sort(m_begin, m_end);
        print(PRI1, "sort_operation_impl::try_start(): end sorting\n");
        print(PRI1, "sort_operation_impl::try_start(): operation.completed()\n");
        operation.completed();
        print(PRI1, "sort_operation_impl::try_start(): end\n");
        break;
    }

    print(PRI1, "sort_operation_impl::try_start(): return;\n");
    return true;
}

void Sorter::sort_operation_impl::get_result(async_operation_ls_base&)
{
    print(PRI1, "sort_operation_impl::get_result()\n");
}

// Linking errors when p2200-sort.cpp is not included here,
// but added to the add_executable definition in CMakeLists.txt
#include "p2200-sort.cpp"
