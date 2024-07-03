/**
 * @file task_coroutine_handle.h
 * @brief
 *
 * Contains the task class that is used in p0120_coroutine_handle.cpp.
 * 
 * task::promise_type::final_awaiter::await_suspend() returns std::coroutine_handle<>.
 * task::awaiter::await_suspend() returns coroutine_handle<>.
 * 
 * @author Lewis Baker
 */

#ifndef _TASK_COROUTINE_HANDLE_H_
#define _TASK_COROUTINE_HANDLE_H_

#include <stdio.h>
#include <coroutine>
#include <utility>
#include <thread>

using namespace std;

class task {
public:
    class promise_type {
    public:
        task get_return_object() noexcept {
            return task{ coroutine_handle<promise_type>::from_promise(*this) };
        }

        suspend_always initial_suspend() noexcept {
            return {};
        }

        void return_void() noexcept {}

        void unhandled_exception() noexcept {
            std::terminate();
        }

        struct final_awaiter {
            bool await_ready() noexcept {
                return false;
            }
            coroutine_handle<> await_suspend(coroutine_handle<promise_type> h) noexcept {
                if (h.promise().continuation)
                    return h.promise().continuation;
                else
                    return std::noop_coroutine();
            }
            void await_resume() noexcept {}
        };

        final_awaiter final_suspend() noexcept {
            return {};
        }

        coroutine_handle<> continuation;
    };

    task(task&& t) noexcept
        : coro_(std::exchange(t.coro_, {}))
    {}

    ~task() {
        if (coro_)
            if (coro_.done()) {
                coro_.destroy();
                coro_ = { };
            }
            else {
                printf("%p: task::~task(): !coro.done()\n", this);
            }
        else
            printf("%p: task::~task(): coro_ == nullptr\n", this);
    }

    // Added by Johan Vanslembrouck
    void start() {
        if (coro_)
            coro_.resume();
    }

    class awaiter {
    public:
        bool await_ready() noexcept {
            return false;
        }

        coroutine_handle<> await_suspend(coroutine_handle<> continuation) noexcept {
            // Store the continuation in the task's promise so that the final_suspend()
            // knows to resume this coroutine when the task completes.
            coro_.promise().continuation = continuation;

            // Then we resume the task's coroutine, which is currently suspended
            // at the initial-suspend-point (ie. at the open curly brace).
            return coro_;
        }

        void await_resume() noexcept {}
    private:
        friend task;
        explicit awaiter(coroutine_handle<promise_type> h) noexcept
            : coro_(h)
        {}

        coroutine_handle<promise_type> coro_;
    };

    awaiter operator co_await() && noexcept {
        return awaiter{ coro_ };
    }

private:
    explicit task(coroutine_handle<promise_type> h) noexcept
        : coro_(h)
    {}

    coroutine_handle<promise_type> coro_;
};

#endif
