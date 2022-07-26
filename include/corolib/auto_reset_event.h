/**
 * @file auto_reset_event.h
 * @brief
 * Defines an event that can be used to resume a waiting coroutine1 from another coroutine, coroutine2.
 * Instead of coroutine1 co_awaiting on the awaitable returned by coroutine2,
 * coroutine1 can pass an auto_reset_event as argument to coroutine2.
 * coroutine1 then co_awaits the resumption of that event from within coroutine2.
 * When coroutine1 is resumed, the m_ready flag is reset.
 * The behavior is similar to using a semaphore, but this is the "coroutine way".
 *
 * Its name is identical to the class defined in cppcoro/lib/auto_reset_event.hpp.
 * However, the implementation in cppcoro is closer to the implementation of a semaphore as
 * the one in semaphore.h.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _AUTO_RESET_EVENT_H_
#define _AUTO_RESET_EVENT_H_

#include <coroutine>
#include "print.h"

namespace corolib
{
    class auto_reset_event 
	{
    public:
	
        auto_reset_event()
            : m_awaiting(nullptr)
            , m_ready(false)
        {
            print(PRI2, "%p: auto_reset_event::auto_reset_event()\n", this);
        }

        auto_reset_event(const auto_reset_event&) = delete;
        auto_reset_event& operator = (const auto_reset_event&) = delete;

        auto_reset_event(auto_reset_event&& s) noexcept
            : m_awaiting(s.m_awaiting)
            , m_ready(s.m_ready)
        {
            print(PRI2, "%p: auto_reset_event::auto_reset_event(auto_reset_event&& s)\n", this);
            s.m_awaiting = nullptr;
            s.m_ready = false;
        }

        auto_reset_event& operator = (auto_reset_event&& s) noexcept
        {
            print(PRI2, "%p: auto_reset_event::auto_reset_event = (auto_reset_event&& s)\n", this);
            m_awaiting = s.m_awaiting;
            m_ready = s.m_ready;
            s.m_awaiting = nullptr;
            s.m_ready = false;
            return *this;
        }

        void resume()
        {
            print(PRI2, "%p: auto_reset_event::resume(): before m_awaiting.resume();\n", this);
            m_ready = true;
            if (m_awaiting && !m_awaiting.done())
                m_awaiting.resume();
            print(PRI2, "%p: auto_reset_event::resume(): after m_awaiting.resume();\n", this);
        }

        auto operator co_await() noexcept
        {
            class awaiter
            {
            public:
                awaiter(auto_reset_event& are_)
                    : m_are(are_)
                {
                    print(PRI2, "%p: auto_reset_event::awaiter(auto_reset_event& ars_)\n", this);
                }

                bool await_ready()
                {
                    print(PRI2, "%p: auto_reset_event::await_ready(): return false\n", this);
                    return m_are.m_ready;
                }

                void await_suspend(std::coroutine_handle<> awaiting)
                {
                    print(PRI2, "%p: auto_reset_event::await_suspend(std::coroutine_handle<> awaiting)\n", this);
                    m_are.m_awaiting = awaiting;
                }

                void await_resume()
                {
                    print(PRI2, "%p: void auto_reset_event::await_resume()\n", this);
                    m_are.m_ready = false;
                }

            private:
                auto_reset_event& m_are;
            };

            return awaiter{ *this };
        }

    private:
        std::coroutine_handle<> m_awaiting;
        bool m_ready;
    };
}

#endif
