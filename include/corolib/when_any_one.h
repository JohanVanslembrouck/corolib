/**
 * @file when_any_one.h
 * @brief
 * Auxiliary class used in the implementation of when_any, async_operation and async_task.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _WHEN_ANY_ONE_H_
#define _WHEN_ANY_ONE_H_

#include <coroutine>
#include "print.h"

namespace corolib
{
    class when_any_one
    {
    public:
	
        when_any_one()
            : m_awaiting(nullptr)
            , m_completed(false)
        {
            print(PRI2, "%p: when_any_one::when_any_one()\n", this);
        }

        ~when_any_one()
        {
            print(PRI2, "%p: when_any_one::~when_any_one()\n", this);
            m_awaiting = nullptr;
            m_completed = false;
        }

        /**
         * @brief called from await_suspend in when_any
         *
         */
        void set_awaiting(std::coroutine_handle<> awaiting)
        {
            print(PRI2, "%p: when_any_one::set_awaiting()\n", this);
            m_awaiting = awaiting;
        }

        /**
         * @brief called from await_ready in when_any
         *
         */
        bool get_completed()
        {
            print(PRI2, "%p: when_any_one::get_completed()\n", this);
            return m_completed;
        }

        /**
         * @brief called from await_resume in when_any
         *
         */
        bool get_and_reset_completed()
        {
            print(PRI2, "%p: when_any_one::get_and_reset_completed()\n", this);
            bool completed = m_completed;
            m_completed = false;
            return completed;
        }

		/**
         * @brief called from async_operation_base::completed and 
         * from return_value and return_void in the promise_type of async_task
         *
         */
        void completed()
        {
            print(PRI2, "%p: when_any_one::completed()\n", this);
            m_completed = true;
            // Resume the awaiting coroutine
            m_awaiting.resume();
        }

    private:
        std::coroutine_handle<> m_awaiting;
        bool m_completed;
    };
}

#endif
