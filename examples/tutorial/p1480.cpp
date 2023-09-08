/**
 * @file p1480.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/when_all.h>

#include "p1480.h"

async_operation<void> Sorter::start_sorting(auto begin, auto end)
{
    int index = get_free_index();
    print(PRI1, "%p: Sorter::start_sorting(): index = %d\n", this, index);
    async_operation<void> ret{ this, index, false };
    start_sort(index, begin, end);
    print(PRI1, "%p: Sorter::start_sorting(): return ret;\n");
    return ret;
}

void Sorter::start_sort(int idx, auto begin, auto end)
{
    print(PRI1, "%p: Sorter::start_sort(): idx = %d\n", this, idx);

    switch (m_useMode)
    {
    case UseMode::USE_NONE:
        break;
    case UseMode::USE_EVENTQUEUE:
    {
        print(PRI1, "Sorter::start_sort(): begin sorting\n");
        std::sort(begin, end);
        print(PRI1, "Sorter::start_sort(): end sorting\n");
        if (m_eventQueue)
        {
            m_eventQueue->push(
                [this, idx]()
                {
                    completionHandler_v(idx);
                });
        }
        break;
    }
    case UseMode::USE_THREAD:
    {
        std::thread thread1(
            [this, idx, begin, end]() {
                print(PRI1, "Sorter::start_sort(): thread1: begin sorting\n");
                std::sort(begin, end);
                print(PRI1, "Sorter::start_sort(): thread1: end sorting\n");

                print(PRI1, "Sorter::start_sort(): thread1: completionHandler_v(idx = %d)\n", idx);
                completionHandler_v(idx);
                print(PRI1, "Sorter::start_sort(): thread1: return\n");
            });
        thread1.detach();
        break;
    }
    case UseMode::USE_THREAD_QUEUE:
    {
        m_queueSize++;

        std::thread thread1(
            [this, idx, begin, end]() {
                print(PRI1, "Sorter::start_sort(): thread1: begin sorting\n");
                std::sort(begin, end);
                print(PRI1, "Sorter::start_sort(): thread1: end sorting\n");
                if (m_eventQueueThr)
                {
                    m_eventQueueThr->push(
                        [this, idx]()
                        {
                            completionHandler_v(idx);
                        });
                }
                print(PRI1, "Sorter::start_sort(): thread1: return;\n");
            });
        thread1.detach();
        break;
    }
    case UseMode::USE_IMMEDIATE_COMPLETION:
        break;
    }

    print(PRI1, "%p: Sorter::start_sort(): return\n");
}

async_task<void> sortCoroutine(Sorter& sorter, std::vector<int>& values)
{
   print(PRI1, "sortCoroutine: start\n");

   size_t middle{ values.size() / 2 }; // middle element index

   async_operation<void> op1 = sorter.start_sorting(values.begin(), values.begin() + middle);
   async_operation<void> op2 = sorter.start_sorting(values.begin() + middle, values.end());
   print(PRI1, "sortCoroutine: when_all wa(op1, op2);\n");
   when_all wa(op1, op2);
   print(PRI1, "sortCoroutine: co_await wa;\n");
   co_await wa;

   // merge the two sorted sub-vectors
   print(PRI1, "sortCoroutine: merging results\n");
   std::inplace_merge(values.begin(), values.begin() + middle, values.end());

   print(PRI1, "sortCoroutine: co_return\n");
   co_return;
}
