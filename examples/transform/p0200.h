/**
 *  Filename: p0200.h
 *  Description:
 *  This file defines a coroutine type 'task' that will be used in all p02XX.cpp and p02XXtrf.cpp examples.
 * 
 *  The definition has to be tailored in two ways:
 * 
 *  1) By setting exactly one of the following three compiler directives to 1 in the .cpp files:
 *          AWAIT_SUSPEND_RETURNS_VOID
 *          AWAIT_SUSPEND_RETURNS_BOOL
 *          AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE
 *     This will select one of the three implementations of await_suspend(std::coroutine_handle<>).
 * 
 *  2) By setting (or not) the following compiler directive to 1:
 *          USE_FINAL_AWAITER
 *     If set to 1, final_supend() will return final_awaiter, otherwise it will return std::suspend_always.
 *     
 *     In this file final_awaiter has the same behavior as std::suspend_always.
 *     See p0300.h for a definition of final_awaiter with several variants.
 * 
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P0200_H
#define _P0200_H

using namespace std;

#include "print.h"
#include "tracker.h"

struct task : private coroutine_tracker {

    struct promise_type;
    using coro_handle = std::coroutine_handle<promise_type>;

    task(coro_handle coroutine)
        : m_coroutine(coroutine) {
        print(PRI2, "task::task(coro_handle coroutine)\n");
    }

    ~task() {
        print(PRI2, "task::~task()\n");
        if (m_coroutine) {
            print(PRI2, "task::~task(): m_coroutine.done() = %d\n", m_coroutine.done());
            if (m_coroutine.done())
                m_coroutine.destroy();
        }
    }

    task() = delete;
    task(task const&) = delete;
    task& operator= (task const&) = delete;

    task(task&& other) noexcept
        : m_coroutine(other.m_coroutine) {
        print(PRI2, "task::task(task&& other)\n");
        other.m_coroutine = {};
    }

    task& operator= (task&& other) noexcept {
        print(PRI2, "task::operator= (task&& other)\n");
        if (&other != this) {
            m_coroutine = other.m_coroutine;
            other.m_coroutine = {};
        }
        return *this;
    }

    int get() {
        print(PRI2, "task::get()\n");
        if (m_coroutine)
            if (m_coroutine.promise().m_ready)
                return m_coroutine.promise().m_value;
        return -3;
    }

    struct awaiter {
        explicit awaiter(std::coroutine_handle<promise_type> coro) noexcept
            : m_coroutine(coro)
        { 
            print(PRI2, "awaiter::awaiter(std::coroutine_handle<promise_type> coro)\n");
        }

        bool await_ready() noexcept {
            print(PRI2, "task::awaiter::await_ready()\n");
            return m_coroutine.promise().m_ready;
        }
#if AWAIT_SUSPEND_RETURNS_VOID
        void await_suspend(std::coroutine_handle<> awaiting) noexcept {
            print(PRI2, "task::awaiter::await_suspend(std::coroutine_handle<> awaiting)\n");
            m_coroutine.promise().m_awaiting = awaiting;
        }
#endif
#if AWAIT_SUSPEND_RETURNS_BOOL
        bool await_suspend(std::coroutine_handle<> awaiting) noexcept {
            print(PRI2, "task::awaiter::await_suspend(std::coroutine_handle<> awaiting)\n");
            m_coroutine.promise().m_awaiting = awaiting;
            return !m_coroutine.promise().m_ready;
        }
#endif
#if AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE
        std::coroutine_handle<> await_suspend(std::coroutine_handle<> awaiting) noexcept {
            print(PRI2, "task::awaiter::await_suspend(...): before m_awaitable.m_coroutine.resume();\n");
            m_coroutine.resume();
            print(PRI2, "task::awaiter::await_suspend(...): after m_awaitable.m_coroutine.resume();\n");
            return awaiting;
        }
#endif
        int await_resume() {
            print(PRI2, "task::awaiter::await_resume()\n");
            return m_coroutine.promise().m_value;
        }

    private:
        std::coroutine_handle<promise_type> m_coroutine;
    };

    awaiter operator co_await() && noexcept {
        return task::awaiter{ m_coroutine };
    }

    struct promise_type : private promise_type_tracker {
        using coro_handle = std::coroutine_handle<promise_type>;

        promise_type()
            : m_value(-1)
            , m_ready(false)
            , m_awaiting(nullptr) {
            print(PRI2, "task::promise_type::promise_type()\n");
        }

        ~promise_type() {
            print(PRI2, "task::promise_type::~promise_type()\n");
            m_ready = false;
            m_value = -2;
        }

        auto get_return_object() {
            print(PRI2, "task::promise_type::get_return_object()\n");
            return coro_handle::from_promise(*this);
        }

        static task get_return_object_on_allocation_failure() {
            print(PRI2, "task::promise_type::get_return_object_on_allocation_failure()\n");
            throw std::bad_alloc();
        }

        auto initial_suspend() {
            print(PRI2, "task::promise_type::initial_suspend()\n");
            return std::suspend_never{};
        }

        struct final_awaiter : public final_awaiter_tracker {
            final_awaiter(promise_type& pr)
                : m_pr(pr)
            {}
            
            bool await_ready() const noexcept {
                print(PRI2, "task::promise_type::final_awaiter::await_ready()\n");
                return m_pr.m_ready;
            }

            void await_suspend(coro_handle) noexcept {
                print(PRI2, "task::promise_type::final_awaiter::await_suspend()\n");
                print(PRI2, "task::promise_type::final_awaiter::await_suspend(): m_ready = %d\n", m_pr.m_ready);
                print(PRI2, "task::promise_type::final_awaiter::await_suspend(): m_value = %d\n", m_pr.m_value);
            }

            void await_resume() noexcept {
                print(PRI2, "task::promise_type::final_awaiter::await_resume()\n");
            }

            promise_type& m_pr;
        };

        auto final_suspend() noexcept {
            print(PRI2, "task::promise_type::final_suspend()\n");
#if USE_FINAL_AWAITER
            return final_awaiter{ *this };
#else
            return std::suspend_always{};
#endif
        }

        void unhandled_exception() {
            print(PRI2, "task::promise_type::unhandled_exception()\n");
            std::terminate();
        }

        void return_value(int v) {
            print(PRI2, "task::promise_type::return_value(int v)\n");
            m_value = v;
            m_ready = true;
            if (m_awaiting)
                m_awaiting.resume();
        }

        int m_value;
        bool m_ready;
        std::coroutine_handle<> m_awaiting;
    };

    coro_handle m_coroutine;
};

#endif
