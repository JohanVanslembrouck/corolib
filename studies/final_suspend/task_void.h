/**
 * @file task_void.h
 * @brief
 * 
 * Contains the task class that is common to p0000_void.cpp and p0110_void.cpp.
 * 
 * task::promise_type::final_awaiter::await_suspend() returns void.
 * task::awaiter::await_suspend() returns void.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _TASK_VOID_H_
#define _TASK_VOID_H_

#include <coroutine>

#include "tracker.h"
#include "print.h"

using namespace std;

class task : private coroutine_tracker {
public:
    class promise_type : private promise_type_tracker {
    public:
        promise_type() {
            print(PRI2, "%p: promise_type::promise_type()\n", this);
        }

        ~promise_type() {
            print(PRI2, "%p: promise_type::~promise_type()\n", this);
        }

        task get_return_object() noexcept {
            return task{ coroutine_handle<promise_type>::from_promise(*this) };
        }

        suspend_always initial_suspend() noexcept {
            return {};
        }

        void return_value(int v) noexcept {
            value = v;
        }

        void unhandled_exception() noexcept {
            print(PRI2, "%p: promise_type::unhandled_exception()\n", this);
            std::terminate();
        }

        struct final_awaiter {
            bool await_ready() noexcept {
                return false;
            }
            void await_suspend(coroutine_handle<promise_type> h) noexcept {
                if (h.promise().continuation)
                    h.promise().continuation.resume();
            }
            void await_resume() noexcept {}
        };

        final_awaiter final_suspend() noexcept {
            return {};
        }

        coroutine_handle<> continuation{ nullptr };
        int value{ 0 };
    };
#if 0
    task(task&& t) noexcept
        : coro_(std::exchange(t.coro_, {}))
    {}
#endif
    ~task() {
        print(PRI2, "%p: task::~task()\n", this);
        if (coro_)
            if (coro_.done()) {
                coro_.destroy();
                coro_ = nullptr;
            }
            else {
                print(PRI2, "%p: task::~task(): !coro.done()\n", this);
            }
        else
            print(PRI2, "%p: task::~task(): coro_ == nullptr\n", this);
    }

    int get_result() {
        print(PRI2, "%p: task::get_result()\n", this);
        if (coro_)
            return coro_.promise().value;
        else
            return -1;
    }

    void start() {
        if (coro_)
            coro_.resume();
    }

    class awaiter {
    public:
        bool await_ready() noexcept {
            return false;
        }

        void await_suspend(coroutine_handle<> continuation) noexcept {
            // Store the continuation in the task's promise so that the final_suspend()
            // knows to resume this coroutine when the task completes.
            coro_.promise().continuation = continuation;

            // Then we resume the task's coroutine, which is currently suspended
            // at the initial-suspend-point (ie. at the open curly brace).
            coro_.resume();
        }

        int await_resume() noexcept {
            if (coro_) {
                try {
                    return coro_.promise().value;
                }
                catch (...) {
                    return -1;
                }
            }
            else
                return -1;
        }

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
    {
        print(PRI2, "%p: task::task(...)\n", this);
    }

    coroutine_handle<promise_type> coro_{ nullptr };
};

#endif
