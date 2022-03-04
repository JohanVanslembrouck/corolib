/**
 *  Filename: mini0.h
 *  Description:
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 *
 */

#ifndef _MINI0_H_
#define _MINI0_H_

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

struct mini0
{
    std::coroutine_handle<> m_awaiting;

    void resume()
    {
        print(PRI2, "%p: mini::resume(): before m_awaiting.resume();\n", this);
        if (!m_awaiting.done())
            m_awaiting.resume();
        print(PRI2, "%p: mini::resume(): after m_awaiting.resume();\n", this);
    }

    auto operator co_await() noexcept
    {
        class awaiter
        {
        public:

            awaiter(mini0& mini_) :
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

            void await_resume()
            {
                print(PRI2, "%p: void mini::await_resume()\n", this);
            }

        private:
            mini0& m_mini;
        };

        return awaiter{ *this };
    }
};

#endif
