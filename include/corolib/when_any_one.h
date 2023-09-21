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
    enum class when_any_one_status
    {
        NOT_COMPLETED = 0,
        NEWLY_COMPLETED,
        COMPLETED_PROCESSED,
    };

    class when_any_one
    {
    public:
	
        when_any_one()
            : m_awaiting(nullptr)
            , m_completed(false)
            , m_completion_status(when_any_one_status::NOT_COMPLETED)
        {
            print(PRI2, "%p: when_any_one::when_any_one()\n", this);
        }

        ~when_any_one()
        {
            print(PRI2, "%p: when_any_one::~when_any_one()\n", this);
            m_awaiting = nullptr;
            m_completed = false;
            m_completion_status = when_any_one_status::NOT_COMPLETED;
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
         * @brief called from await_ready in when_any
         *
         */
        when_any_one_status get_completion_status()
        {
            print(PRI2, "%p: when_any_one::get_completion_status()\n", this);
            return m_completion_status;
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

        bool get_and_mark_as_completed()
        {
            print(PRI2, "%p: when_any_one::get_and_mark_as_completed()\n", this);
            m_completion_status = when_any_one_status::COMPLETED_PROCESSED;
            return get_and_reset_completed();
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
            m_completion_status = when_any_one_status::NEWLY_COMPLETED;
            // Resume the awaiting coroutine
            m_awaiting.resume();
        }

    private:
        std::coroutine_handle<> m_awaiting;
        bool m_completed;
        when_any_one_status m_completion_status;
    };
}

#endif
