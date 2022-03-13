/**
 * @file wait_any_one.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _WAIT_ANY_ONE_H_
#define _WAIT_ANY_ONE_H_

#include <coroutine>
#include "print.h"

namespace corolib
{
    struct wait_any_one
    {
        wait_any_one()
            : m_awaiting(nullptr)
            , m_completed(false)
        {
            print(PRI2, "%p: wait_any_one::wait_any_one()\n", this);
        }

        ~wait_any_one()
        {
            print(PRI2, "%p: wait_any_one::~wait_any_one()\n", this);
            m_awaiting = nullptr;
            m_completed = false;
        }

        void set_awaiting(std::coroutine_handle<> awaiting)
        {
            print(PRI2, "%p: wait_any_one::set_awaiting()\n", this);
            m_awaiting = awaiting;
        }

        bool get_completed()
        {
            print(PRI2, "%p: wait_any_one::get_completed()\n", this);
            return m_completed;
        }

        bool get_and_reset_completed()
        {
            print(PRI2, "%p: wait_any_one::get_and_reset_completed()\n", this);
            bool completed = m_completed;
            m_completed = false;
            return completed;
        }

        void completed()
        {
            print(PRI2, "%p: wait_any_one::completed()\n", this);
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
