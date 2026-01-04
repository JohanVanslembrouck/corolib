/**
 * @file mini_awaiter_cl.h
 * @brief
 * Defines a simple awaitable. Uses the corolib print function.
 *
 * @author Johan Vanslembrouck
 */

#ifndef _MINI_AWAITER_CL_H_
#define _MINI_AWAITER_CL_H_

#include <coroutine>

#include <corolib/print.h>

using namespace corolib;

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
        print(PRI3, "%p: mini_awaiter::resume() -> void: enter\n", this);
        if (!m_awaiting.done()) {
            print(PRI3, "%p: mini_awaiter::resume() -> void: before m_awaiting.resume();\n", this);
            m_awaiting.resume();
            print(PRI3, "%p: mini_awaiter::resume() -> void: after m_awaiting.resume();\n", this);
        }
        print(PRI3, "%p: mini_awaiter::resume() -> void: leave\n", this);
    }

    auto operator co_await() noexcept
    {
        class awaiter
        {
        public:

            awaiter(mini_awaiter& mini_) :
                m_mini(mini_)
            {
                print(PRI3, "%p: mini_awaiter::awaiter::awaiter(...)\n", this);
            }

            bool await_ready()
            {
                print(PRI3, "%p: mini_awaiter::awaiter::await_ready() -> bool: return false;\n", this);
                return false;
            }

            void await_suspend(std::coroutine_handle<> awaiting)
            {
                print(PRI3, "%p: mini_awaiter::awaiter::await_suspend() -> void\n", this);
                m_mini.m_awaiting = awaiting;
            }

            void await_resume()
            {
                print(PRI3, "%p: mini_awaiter::awaiter::await_resume() -> void\n", this);
            }

        private:
            mini_awaiter& m_mini;
        };

        return awaiter{ *this };
    }
};

#endif
