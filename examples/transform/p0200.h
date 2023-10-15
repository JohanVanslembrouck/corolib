/**
 *  Filename: p0200.h
 *  Description
 * 
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P0200_H
#define _P0200_H

using namespace std;

#include "print.h"

struct task
{
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

    task() = default;
    task(task const&) = delete;
    task& operator= (task const&) = delete;

    task(task&& other)
        : m_coroutine(other.m_coroutine) {
        print(PRI2, "task::task(task&& other)\n");
        //other.m_coroutine = nullptr;
    }

    task& operator= (task&& other) {
        print(PRI2, "task::operator= (task&& other)\n");
        if (&other != this) {
            m_coroutine = other.m_coroutine;
            //other.m_coroutine = nullptr;
        }
        return *this;
    }

    int get() {
        print(PRI2, "task::get()\n");
        if (m_coroutine)
            return m_coroutine.promise().m_value;
        return -1;
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
        void await_suspend(std::coroutine_handle<> awaiting) noexcept {
            print(PRI2, "task::awaiter::await_suspend(std::coroutine_handle<> awaiting)\n");
            m_coroutine.promise().m_awaiting = awaiting;
        }
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

    // defined in template<typename T> struct task
    struct promise_type
    {
        using coro_handle = std::coroutine_handle<promise_type>;

        promise_type()
            : m_value(0)
            , m_ready(false)
            , m_awaiting(nullptr) {
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

        auto final_suspend() noexcept {
            print(PRI2, "task::promise_type::final_suspend()\n");
            return std::suspend_always{};
        }

        void unhandled_exception() {
            print(PRI2, "task::promise_type::unhandled_exception()\n");
            std::terminate();
        }

        void return_value(int v) {
            print(PRI2, "task::promise_type::return_value(int v)\n");
            m_value = v;
            m_ready = true;
            if (m_awaiting) m_awaiting.resume();
        }

        int m_value;
        bool m_ready;
        std::coroutine_handle<> m_awaiting;
    };

    coro_handle m_coroutine;
};


#endif
