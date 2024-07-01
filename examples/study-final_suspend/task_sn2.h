/**
 * @file task_sn2.h
 * @brief
 * 
 * Uses eager start.
 *
 * task::promise_type::final_suspend returns std::suspend_never
 * task::awaiter::await_suspend() returns void.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _TASK_SN2_H_
#define _TASK_SN2_H_

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

        void link_task(task* t) {
            mytask = t;
        }

        void return_value(int v) noexcept {
            if (mytask)
                mytask->value = v;
            if (continuation)
                continuation.resume();
        }

        void unhandled_exception() noexcept {
            std::terminate();
        }

        std::suspend_never final_suspend() noexcept {
            return {};
        }

        coroutine_handle<> continuation{ nullptr };
        task* mytask{ nullptr };
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
        return value;
    }

    void start() {
        if (coro_)
            coro_.resume();
    }

    class awaiter {
    public:
        bool await_ready() noexcept {
            return coro_.done();
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
            return mytask_->value;
        }

    private:
        friend task;
        explicit awaiter(coroutine_handle<promise_type> h, task* t) noexcept
            : coro_(h)
            , mytask_(t)
        {}

        coroutine_handle<promise_type> coro_;
        task* mytask_;
    };

    awaiter operator co_await() && noexcept {
        return awaiter{ coro_, this };
    }

private:
    explicit task(coroutine_handle<promise_type> h) noexcept
        : coro_(h)
    {
        print(PRI2, "%p: task::task(...)\n", this);
        coro_.promise().link_task(this);
    }

    coroutine_handle<promise_type> coro_{ nullptr };
    int value{ 0 };
};

#endif
