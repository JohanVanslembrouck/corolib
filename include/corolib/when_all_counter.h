/**
 * @file when_all_counter.h
 * @brief
 * Auxiliary class used in the implementation of when_all, async_operation and async_task.
 *
 * when_all_counter is passed a counter (of type int) to decrement and the coroutine_handle of a coroutine
 * that it has to resume when the counter drops to 0.
 * The same when_all_counter object is passed to several async_operation or async_task objects
 * that each decrement the counter 
 * when (in case of async_operation) the asynchronous operation completes
 * or (in case of async_task) the coroutine co_returns.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _WHEN_ALL_COUNTER_H_
#define _WHEN_ALL_COUNTER_H_

#include <coroutine>
#include "print.h"

namespace corolib
{
    class when_all_counter
    {
    public:
	
        when_all_counter(int nr)
            : m_awaiting(nullptr)
            , m_nr(nr)
        {
            print(PRI2, "%p: when_all_counter::when_all_counter(%d)\n", this, nr);
        }

        /**
         * @brief called from await_suspend in when_all
         *
         */
        void set_awaiting(std::coroutine_handle<> awaiting)
        {
            print(PRI2, "%p: when_all_counter::set_awaiting()\n", this);
            m_awaiting = awaiting;
        }

        /**
         * @brief called from await_ready in when_all
         *
         */
        int get_counter()
        {
            print(PRI2, "%p: when_all_counter::get_counter(): returns %d\n", this, m_nr);
            return m_nr;
        }

        void increment()
        {
            print(PRI2, "%p: when_all_counter::increment(): m_nr = %d\n", this, m_nr);
            m_nr++;
        }

        /**
         * @brief called from async_operation_base::completed and 
         * from return_value and return_void in the promise_type of async_task
         *
         */
        std::coroutine_handle<> completed()
        {
            print(PRI2, "%p: when_all_counter::completed(): m_nr = %d\n", this, m_nr);
            m_nr--;
            if (m_nr == 0)
            {
                print(PRI2, "%p: when_all_counter::completed(): all replies received\n", this);
                return m_awaiting;
            }
            return std::noop_coroutine();
        }

    private:
        std::coroutine_handle<> m_awaiting;
        int m_nr;
    };
}

#endif
