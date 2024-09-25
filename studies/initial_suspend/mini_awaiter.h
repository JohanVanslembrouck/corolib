/**
 * @file mini_awaiter.h
 * @brief
 * Defines a simple awaitable.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _MINI_AWAITER_H_
#define _MINI_AWAITER_H_

#include <coroutine>

#include "tracker1.h"

/**
 * @brief mini_awaiter mini0 defines a simple awaitable type with two member functions:
 * 1) function resume() resumes the coroutine that co_awaits a mini0 object.
 * 2) operator co_await allows a coroutine to co_await a mini0 object.
 */
struct mini_awaiter
{
    std::coroutine_handle<> m_awaiting;

    void resume()
    {
        if (!m_awaiting.done()) {
            tracker1_obj.nr_resumptions++;
            m_awaiting.resume();
        }
    }

    auto operator co_await() noexcept
    {
        class awaiter
        {
        public:

            awaiter(mini_awaiter& mini_) :
                m_mini(mini_)
            {}

            bool await_ready()
            {
                return false;
            }

            void await_suspend(std::coroutine_handle<> awaiting)
            {
                m_mini.m_awaiting = awaiting;
            }

            void await_resume() { }

        private:
            mini_awaiter& m_mini;
        };

        return awaiter{ *this };
    }
};

#endif
