/**
 * @file task.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#pragma once

#include <coroutine>
#include <utility>
#include <exception>        // defines std::terminate() for gcc

#include "corolib/print.h"

using namespace std;
using namespace corolib;

class task {
public:
    class promise_type {
    public:
        task get_return_object() noexcept {
            return task{ coroutine_handle<promise_type>::from_promise(*this) };
        }

        suspend_never initial_suspend() noexcept {
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
            void await_suspend(coroutine_handle<promise_type> h) noexcept {
                if (h.promise().continuation) {
                    h.promise().continuation.resume();
                }
            }
            void await_resume() noexcept {}
        };

        final_awaiter final_suspend() noexcept {
            return {};
        }

        coroutine_handle<> continuation = nullptr;
    };

    task(task&& t) noexcept
        : coro_(std::exchange(t.coro_, {}))
    {
    }

    ~task() {
        if (coro_)
            if (coro_.done()) {
                coro_.destroy();
                coro_ = { };
            }
            else {
                print(PRI1, "%p: task::~task(): !coro.done()\n", this);
            }
        else
            print(PRI1, "%p: task::~task(): coro_ == nullptr\n", this);
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
        }

        void await_resume() noexcept {}

    private:
        friend task;
        explicit awaiter(coroutine_handle<promise_type> h) noexcept
            : coro_(h)
        {
        }

        coroutine_handle<promise_type> coro_;
    };

    awaiter operator co_await() && noexcept {
        return awaiter{ coro_ };
    }

    void start() {

    }

private:
    explicit task(coroutine_handle<promise_type> h) noexcept
        : coro_(h)
    {
    }

    coroutine_handle<promise_type> coro_;
};
