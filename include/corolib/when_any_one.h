/**
 * @file when_any_one.h
 * @brief
 * Auxiliary class used in the implementation of when_any, async_operation and async_task.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _WHEN_ANY_ONE_H_
#define _WHEN_ANY_ONE_H_

#include <coroutine>

#if USE_IN_MT_APPS
#include <atomic>
#endif

#include "print.h"

namespace corolib
{
    class when_any_one
    {
    private:
        enum class when_any_one_status : char
        {
            NOT_COMPLETED = 0,
            NEWLY_COMPLETED,
            COMPLETED_PROCESSED,
        };

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

        when_any_one(const when_any_one&) = default;
        when_any_one(when_any_one&&) noexcept = default;

        when_any_one& operator = (const when_any_one&) = delete;
        when_any_one& operator = (when_any_one&&) noexcept = default;

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
#if USE_IN_MT_APPS
            print(PRI2, "%p: when_any_one::get_completed(): m_completed = %d\n", this, m_completed.load());
#else
            print(PRI2, "%p: when_any_one::get_completed(): m_completed = %d\n", this, m_completed);
#endif
            return m_completed;
        }

        /**
         * @brief called from await_resume in when_any
         *
         */
        bool get_and_mark_as_completed()
        {
            print(PRI2, "%p: when_any_one::get_and_mark_as_completed()\n", this);
#if USE_IN_MT_APPS
            bool expected = true;
            if (m_completed.compare_exchange_strong(expected, false)) {
                m_completion_status = when_any_one_status::COMPLETED_PROCESSED;
                return true;
            }
            return false;
#else
            bool completed = m_completed;
            if (m_completed) {
                m_completed = false;
                m_completion_status = when_any_one_status::COMPLETED_PROCESSED;
            }
            return completed;
#endif
        }

        /**
         * @brief called from async_operation_base::completed() and 
         * from return_value() and return_void() in the promise_type of async_task
         *
         */
        std::coroutine_handle<> completed()
        {
            print(PRI2, "%p: when_any_one::completed()\n", this);
            m_completion_status = when_any_one_status::NEWLY_COMPLETED;
            m_completed = true;
            return m_awaiting;
        }

        void set_completed(bool completed)
        {
            m_completed = completed;
        }

    private:
        std::coroutine_handle<> m_awaiting;
#if USE_IN_MT_APPS
        std::atomic<bool> m_completed;
#else
        bool m_completed;
#endif
        when_any_one_status m_completion_status;
    };
}

#endif
