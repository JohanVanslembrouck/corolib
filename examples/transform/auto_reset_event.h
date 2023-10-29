/**
 *  Filename: auto_reset_event.h
 *  Description:
 *  This file defines a type that is used to resume the coroutine (typically) at the top of the call stack.
 *  This coroutine has to co_await an object of type auto_reset_event.
 *  When another function (or coroutine) calls resume() on the object, the awaiting coroutine will be resumed.
 *  This function will typically be the main() function.
 * 
 *  The name "auto_reset" comes from the fact that the ready flag is set to false in funcion await_resume().
 *  This means that every co_await on an object of this type requires the resume() function to be called
 *  to allow the awaiting coroutine to resume.
 * 
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _AUTO_RESET_EVENT_H_
#define _AUTO_RESET_EVENT_H_

#include "print.h"

struct auto_reset_event {

    std::coroutine_handle<> m_awaiting;

    auto_reset_event()
        : m_awaiting(nullptr)
        , m_ready(false) {
        print(PRI2, "auto_reset_event::auto_reset_event()\n");
    }

    auto_reset_event(const auto_reset_event&) = default;
    auto_reset_event& operator = (const auto_reset_event&) = default;

    auto_reset_event(auto_reset_event&& s) noexcept
        : m_awaiting(s.m_awaiting)
        , m_ready(s.m_ready) {
        print(PRI2, "auto_reset_event::auto_reset_event(auto_reset_event&& s)\n");
        s.m_awaiting = nullptr;
        s.m_ready = false;
    }

    auto_reset_event& operator = (auto_reset_event&& s) noexcept {
        print(PRI2, "auto_reset_event::operator = (auto_reset_event&& s)\n");
        m_awaiting = s.m_awaiting;
        m_ready = s.m_ready;
        s.m_awaiting = nullptr;
        s.m_ready = false;
        return *this;
    }

    void resume() {
        print(PRI2, "auto_reset_event::resume()\n");
        m_ready = true;
        if (m_awaiting && !m_awaiting.done())
            m_awaiting.resume();
    }

    struct awaiter
    {
        awaiter(auto_reset_event& are_) : m_are(are_) {
            print(PRI2, "auto_reset_event::awaiter::awaiter(auto_reset_event& are_)\n");
        }

        bool await_ready() {
            print(PRI2, "auto_reset_event::awaiter::await_ready()\n");
            return m_are.m_ready;
        }

        void await_suspend(std::coroutine_handle<> awaiting) {
            print(PRI2, "auto_reset_event::awaiter::await_suspend(std::coroutine_handle<> awaiting)\n");
            m_are.m_awaiting = awaiting;
        }

        void await_resume() {
            print(PRI2, "auto_reset_event::awaiter::await_resume()\n");
            m_are.m_ready = false;
        }

    private:
        auto_reset_event& m_are;
    };

    auto operator co_await() noexcept
    {
        return awaiter(*this);
    }

    bool m_ready;
};

#endif
