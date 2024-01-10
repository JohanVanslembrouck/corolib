/**
 *  Filename: p0300lnk.h
 *  Description:
 *  This file defines a coroutine type 'task' that will be used in all p03XX.cpp and p03XXtrf.cpp examples.
 * 
 * 
 * 
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P0300LNK_H
#define _P0300LNK_H

using namespace std;

#define USE_FINAL_AWAITER 1

#include "print.h"
#include "tracker.h"

struct task : private coroutine_tracker {

    struct promise_type;
    using coro_handle = std::coroutine_handle<promise_type>;

    task(coro_handle coroutine)
        : m_coroutine(coroutine)
        , m_value(0)
        , m_ready(false) {
        print(PRI2, "task::task(coro_handle coroutine)\n");
        m_coroutine.promise().link_coroutine_object(this);
    }

    ~task() {
        print(PRI2, "task::~task()\n");
        m_value = 0;
        m_ready = false;
        if (m_coroutine) {
            print(PRI2, "task::~task(): m_coroutine.done() = %d\n", m_coroutine.done());
            if (m_coroutine.done())
                m_coroutine.destroy();
            m_coroutine.promise().unlink_coroutine_object();
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

    int get() {
        print(PRI2, "task::get()\n");
        if (m_ready)
            return m_value;
        return -1;
    }

    struct awaiter {
        explicit awaiter(task* tsk) noexcept
            : m_task(*tsk)
        {
            print(PRI2, "awaiter::awaiter(std::coroutine_handle<promise_type> coro)\n");
        }

        bool await_ready() noexcept {
            print(PRI2, "task::awaiter::await_ready()\n");
            return m_task.m_ready;
        }

        void await_suspend(std::coroutine_handle<> awaiting) noexcept {
            print(PRI2, "task::awaiter::await_suspend(std::coroutine_handle<> awaiting)\n");
            m_task.m_coroutine.promise().m_awaiting = awaiting;
        }

        int await_resume() {
            print(PRI2, "task::awaiter::await_resume()\n");
            return m_task.m_value;
        }

    private:
        std::coroutine_handle<promise_type> m_coroutine;
        task& m_task;
    };

    awaiter operator co_await() && noexcept {
        return task::awaiter{ this };
    }

    struct promise_type : private promise_type_tracker {
        using coro_handle = std::coroutine_handle<promise_type>;

        promise_type()
            : m_awaiting(nullptr)
            , m_coroutine_object(nullptr)
            , m_coroutine_valid(false) {
            print(PRI2, "task::promise_type::promise_type()\n");
        }

        ~promise_type() {
            print(PRI2, "task::promise_type::~promise_type()\n");
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
            
            bool await_ready() const noexcept {
                print(PRI2, "task::promise_type::final_awaiter::await_ready()\n");
                return false;
            }
#if FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_VOID
            void await_suspend(coro_handle h) noexcept {
                print(PRI2, "task::promise_type::final_awaiter::await_suspend(): h.promise().m_coroutine_object->m_ready = %d\n", h.promise().m_coroutine_object->m_ready);
            }
#endif
#if FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL
            bool await_suspend(coro_handle h) noexcept {
                print(PRI2, "task::promise_type::final_awaiter::await_suspend(): h.promise().m_coroutine_object = %p\n", h.promise().m_coroutine_object);
                if (h.promise().m_coroutine_object) {
                    print(PRI2, "task::promise_type::final_awaiter::await_suspend(): h.promise().m_coroutine_object->m_ready = %d\n", h.promise().m_coroutine_object->m_ready);
                    return !h.promise().m_coroutine_object->m_ready;
                }
                else
                    return false;
            }
#endif
#if FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE
            std::coroutine_handle<> await_suspend(coro_handle h) noexcept {
                print(PRI2, "task::promise_type::final_awaiter::await_suspend(): h.promise().m_awaiting = %p\n", h.promise().m_awaiting);
                if (h.promise().m_awaiting)
                    return h.promise().m_awaiting;
                else
                    return std::noop_coroutine();
            }
#endif
            void await_resume() noexcept {
                print(PRI2, "task::promise_type::final_awaiter::await_resume()\n");
            }

        };

        auto final_suspend() noexcept {
            print(PRI2, "task::promise_type::final_suspend()\n");
#if USE_FINAL_AWAITER
            return final_awaiter{};
#else
            return std::suspend_always{};
#endif
        }

        void unhandled_exception() {
            print(PRI2, "task::promise_type::unhandled_exception()\n");
            std::terminate();
        }

        void return_value(int v) {
            print(PRI2, "task::promise_type::return_value(int v = %d)\n", v);
            if (m_coroutine_object) {
                m_coroutine_object->m_value = v;
                m_coroutine_object->m_ready = true;
            }
#if FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_VOID || FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL
            if (m_awaiting)
                m_awaiting.resume();
#endif
        }

        void link_coroutine_object(task* coroutine_object)
        {
            print(PRI2, "task::promise_type::link_coroutine_object(task* coroutine_object = %p)\n", coroutine_object);
            m_coroutine_object = coroutine_object;
            m_coroutine_valid = true;
        }

        void unlink_coroutine_object()
        {
            m_coroutine_valid = false;
        }

        std::coroutine_handle<> m_awaiting;
        task* m_coroutine_object;
        bool m_coroutine_valid;
    };

    coro_handle m_coroutine;
    int m_value;
    bool m_ready;
};

#endif
