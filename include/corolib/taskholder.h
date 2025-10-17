/**
* @file taskholder.h
* @brief
* class TaskHolder allows storing NR_TASKS async_task<TYPE> objects in an array,
* looking up a free entry in the array to store an async_task<TYPE> object,
* and waiting for the completion of any of, or all, stored objects if there aren't any free entries anymore.
* 
* Class TaskHolder is typically used by a server application to store async_task<TYPE> objects that are returned
* from a coroutine launched to handle the communication with a client.
* 
* @author Johan Vanslembrouck
*/

#ifndef _TASKHOLDER_H_
#define _TASKHOLDER_H_

#include "print.h"
#include "async_task.h"
#include "when_all.h"
#include "when_any.h"

using namespace corolib;

namespace corolib
{
    template<int NR_TASKS, typename TYPE>
    class TaskHolder
    {
    public:
        enum class usage_status
        {
            UNUSED = 0,
            RESERVED,
            USED
        };

        void init()
        {
            m_index = -1;
            for (int i = 0; i < NR_TASKS; ++i)
            {
                usages[i] = usage_status::UNUSED;
            }
        }

        TaskHolder()
        {
            init();
        }

        int get_free_index()
        {
            for (int i = 0; i < NR_TASKS; i++)
            {
                m_index = (m_index + 1) & (NR_TASKS - 1);

                if (usages[m_index] == usage_status::UNUSED)
                {
                    // Found free entry
                    clprint(PRI2, "TaskHolder::get_free_index() returns %d\n", m_index);
                    usages[m_index] = usage_status::RESERVED;
                    return m_index;
                }
            }
            // No more free entries
            return -1;
        }

        /**
        * @brief TaskHolder::wait_free_index
        * This coroutine looks for a free entry.
        * If one is found, wait_free_index returns the index to the free entry immediately.
        * If there aren't any free entries anymore, wait_free_index waits either
        * 1) for all tasks to finish (wait_all == true)
        * or 2) for any task to finish (wait_all == false) and this entry is returned as the free index.
        * See wait_free_index_all for an alternative waiting for all tasks to finixh.
        */
        async_task<int> wait_free_index(bool wait_all = false)
        {
            int i = get_free_index();
            if (i == -1)
            {
                // No free entry anymore. Wait for all running tasks to finish.
                //inspect();

                if (wait_all)
                {
                    clprint(PRI2, "TaskHolder::wait_free_index: when_allT<async_task<TYPE>> wa(tasks, NR_TASKS)\n");
                    when_allT<async_task<TYPE>> wa(tasks, NR_TASKS);

                    clprint(PRI2, "TaskHolder::wait_free_index: before co_await wa;\n");
                    co_await wa;
                    clprint(PRI2, "TaskHolder::wait_free_index: after co_await wa;\n");

                    //inspect();

                    init();
                    i = get_free_index();
                }
                else
                {
                    clprint(PRI2, "TaskHolder::wait_free_index: when_anyT<async_task<TYPE>> wa(tasks, NR_TASKS)\n");
                    when_anyT<async_task<TYPE>> wa(tasks, NR_TASKS);

                    clprint(PRI2, "TaskHolder::wait_free_index: before co_await wa;\n");
                    i = co_await wa;
                    clprint(PRI2, "TaskHolder::wait_free_index: after co_await wa; i = %d\n", i);

                }
                //inspect();

                if (i == -1)
                {
                    clprint(PRI2, "TaskHolder::wait_free_index: could not find free index!!!\n");
                }
            }
            co_return i;
        }

        /**
         * @brief TaskHolder::wait_all_finished
         * This coroutine waits until all tasks have finixhed.
         * Its code also appears in wait_free_index_all, so this latter coroutine
         * could call wait_free_index_all instead.
         */
        async_task<void> wait_all_finished()
        {
            clprint(PRI2, "TaskHolder::wait_all_finished: wait for all tasks to finish\n");
            inspect();

            clprint(PRI2, "TaskHolder::wait_all_finished: when_allT<async_task<int>> wa(tasks, NR_TASKS)\n");
            when_allT<async_task<int>> wa(tasks, NR_TASKS);

            clprint(PRI2, "TaskHolder::wait_all_finished: before co_await wa;\n");
            co_await wa;
            clprint(PRI2, "TaskHolder::wait_all_finished: after co_await wa;\n");

            inspect();
        }

        void assign(int index, async_task<TYPE>&& task)
        {
            usages[index] = usage_status::USED;
            tasks[index] = std::move(task);
        }

        void inspect()
        {
            for (int i = 0; i < NR_TASKS; i++)
            {
                clprint(PRI2, "tasks[%d].is_ready() = %d\n", i, tasks[i].is_ready());
            }
        }

    public:
        usage_status usages[NR_TASKS];
        async_task<TYPE> tasks[NR_TASKS];
        int m_index;
    };
}

#endif
