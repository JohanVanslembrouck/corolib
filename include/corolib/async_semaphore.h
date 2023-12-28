/**
 * @file async_semaphore.h
 * @brief async_semaphore is the coroutine version of a semaphore using a condition_variable + mutex (see semaphore.h)
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _ASYNC_SEMAPHORE_H_
#define _ASYNC_SEMAPHORE_H_

#include <queue>

#include "commservice.h"
#include "async_operation.h"

using namespace corolib;

namespace corolib
{
    class async_semaphore : private CommService
    {
    public:
        async_semaphore(unsigned int count = 0) : m_count(count) { }

        void reset()
        {
            m_count = 0;
        }

        void signal()
        {
            ++m_count;
            if (!m_waiters_queue.empty())
            {
                // Resume the first waiter
                int idx = m_waiters_queue.front();
                m_waiters_queue.pop();
                complete_waiter(idx);
            }
        }

        async_operation<void> wait()
        {
            int index = get_free_index();
            async_operation<void> ret{ this, index };

            if (m_count == 0)
            {
                // Push this operation onto the waiting queue
                print(PRI2, "async_semaphore::wait(): m_waiters_queue.push(%d);\n", index);
                m_waiters_queue.push(index);
            }
            else if (m_count < 0)
            {
                print(PRI1, "async_semaphore::wait(): m_count = %d < 0!\n", m_count);
            }
            else
            {
                --m_count;
                ret.completed();
            }
            
            return ret;
        }

    protected:
        void complete_waiter(int idx)
        {
            print(PRI2, "async_semaphore::complete_waiter(idx = %d)\n", idx);

            async_operation_base* om_async_operation = m_async_operations[idx];
            async_operation<void>* om_async_operation_t =
                static_cast<async_operation<void>*>(om_async_operation);

            if (om_async_operation_t)
            {
                print(PRI2, "async_semaphore::complete_push(%d): om_async_operation_t = %p\n", idx, om_async_operation_t);
                --m_count;
                om_async_operation_t->completed();
            }
            else
            {
                print(PRI1, "async_semaphore::complete_push(idx = %d): om_async_operation_t = nullptr\n", idx);
            }
        }

    private:
        std::queue<int> m_waiters_queue;
        unsigned int m_count;
    };
}

#endif

