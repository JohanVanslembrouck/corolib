/**
 * @file mini1.h
 * @brief
 * Defines a simple awaitable.
 *
 * Compared with mini0.h:
 * - mini1 is a template type
 * - mini1 returns a value
 * - mini1 has an additional set_and_resume member function
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _MINI1_H_
#define _MINI1_H_

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

/**
 * @brief struct mini1 defines a simple awaitable type with three member functions:
 * 1) function resume() resumes the coroutine that co_awaits a mini0 object.
 * 2) function set_and_resume() first sets the return value before it resumes the 
 *    coroutine that co_awaits a mini0 object.
 * 2) operator co_await allows a coroutine to co_await a mini0 object.
 */
template<typename T>
struct mini1
{
    std::coroutine_handle<> m_awaiting;

    void resume()
    {
        print(PRI2, "%p: mini::resume(): before m_awaiting.resume();\n", this);
        m_awaiting.resume();
        print(PRI2, "%p: mini::resume(): after m_awaiting.resume();\n", this);
    }

    void set_and_resume(T value)
    {
        print(PRI2, "%p: mini::resume(): before m_awaiting.resume();\n", this);
        m_value = value;
        m_awaiting.resume();
        print(PRI2, "%p: mini::resume(): after m_awaiting.resume();\n", this);
    }

    auto operator co_await() noexcept
    {
        class awaiter
        {
        public:

            awaiter(mini1& mini_) :
                m_mini(mini_)
            {}

            bool await_ready()
            {
                print(PRI2, "%p: mini::await_ready(): return false\n", this);
                return false;
            }

            void await_suspend(std::coroutine_handle<> awaiting)
            {
                print(PRI2, "%p: mini::await_suspend(std::coroutine_handle<> awaiting)\n", this);
                m_mini.m_awaiting = awaiting;
            }

            T await_resume()
            {
                print(PRI2, "%p: void mini::await_resume()\n", this);
                return m_mini.m_value;
            }

        private:
            mini1& m_mini;
        };

        return awaiter{ *this };
    }

    T m_value{};
};

#endif
