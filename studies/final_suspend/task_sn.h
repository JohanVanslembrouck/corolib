/**
 * @file task_sn.h
 * @brief
 * 
 * Uses lazy start.
 *
 * task::promise_type::final_suspend returns std::suspend_never
 * task::awaiter::await_suspend() returns void.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _TASK_SN_H_
#define _TASK_SN_H_

#include <coroutine>

#include "tracker.h"
#include "suspend.h"
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
            print(PRI3, "%p: promise_type::get_return_object() -> task\n", this);
            return task{ coroutine_handle<promise_type>::from_promise(*this) };
        }

        suspend_always_p initial_suspend() noexcept {
            print(PRI3, "%p: promise_type::initial_suspend() -> suspend_always_p\n", this);
            return {};
        }

        void return_value(int v) noexcept {
            print(PRI3, "%p: promise_type::return_value(%d) -> void: enter\n", this, v);
            value = v;
            if (continuation) {
                print(PRI3, "%p: promise_type::return_value(%d) -> void: before continuation.resume();\n", this, v);
                continuation.resume();
                print(PRI3, "%p: promise_type::return_value(%d) -> void: after continuation.resume();\n", this, v);
            }
            print(PRI3, "%p: promise_type::return_value(%d) -> void: leave\n", this, v);
        }

        void unhandled_exception() noexcept {
            print(PRI3, "%p: promise_type::unhandled_exception() -> void\n", this);
            std::terminate();
        }

        suspend_never_p final_suspend() noexcept {
            print(PRI3, "%p: promise_type::final_suspend() -> suspend_never_p\n", this);
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
#if USE_CORO_DONE_TEST
    ~task() {
        print(PRI2, "%p: task::~task(): test on coro_.done()\n", this);
        if (coro_)
            if (coro_.done()) {
                print(PRI2, "%p: task::~task(): coro_.destroy();\n", this);
                coro_.destroy();
                coro_ = {};
            }
            else {
                print(PRI2, "%p: task::~task(): !coro.done()\n", this);
            }
        else
            print(PRI2, "%p: task::~task(): coro_ == nullptr\n", this);
    }
#else
    ~task() {
        print(PRI2, "%p: task::~task(): no test on coro_.done()\n", this);
        if (coro_) {
            print(PRI2, "%p: task::~task(): coro_.destroy();\n", this);
            coro_.destroy();
            coro_ = {};
        }
        else
            print(PRI2, "%p: task::~task(): coro_ == nullptr\n", this);
    }
#endif

    int get_result() {
        if (coro_) {
            int value = coro_.promise().value;
            print(PRI3, "%p: task::get_result() -> int: return %d;\n", this, value);
            return value;
        }
        else {
            print(PRI3, "%p: task::get_result() -> int: return -1;\n", this);
            return -1;
        }
    }

    void start() {
        print(PRI3, "%p: task::start() -> void\n", this);
        if (coro_) {
            print(PRI3, "%p: task::start() -> void: before coro_.resume();\n", this);
            coro_.resume();
            print(PRI3, "%p: task::start() -> void: after coro_.resume();\n", this);
        }
    }

    class awaiter {
    public:
        bool await_ready() noexcept {
            bool done = coro_.done();
            print(PRI3, "%p: task::awaiter::await_ready() -> bool: return %d;\n", this, done);
            return done;
        }

        void await_suspend(coroutine_handle<> continuation) noexcept {
            print(PRI3, "%p: task::awaiter::await_suspend() -> void: enter\n", this);
            // Store the continuation in the task's promise so that the final_suspend()
            // knows to resume this coroutine when the task completes.
            coro_.promise().continuation = continuation;

            // Then we resume the task's coroutine, which is currently suspended
            // at the initial-suspend-point (ie. at the open curly brace).
            print(PRI3, "%p: task::awaiter::await_suspend() -> void: before coro_.resume();\n", this);
            coro_.resume();
            print(PRI3, "%p: task::awaiter::await_suspend() -> void: after coro_.resume();\n", this);
            print(PRI3, "%p: task::awaiter::await_suspend() -> void: leave\n", this);
        }

        int await_resume() noexcept {
            if (coro_) {
                int value = coro_.promise().value;
                print(PRI3, "%p: task::awaiter::await_resume() -> int: return %d;\n", this, value);
                return value;
            }
            else {
                print(PRI3, "%p: task::awaiter::await_resume() -> int: return -1;\n", this);
                return -1;
            }
        }

    private:
        friend task;
        explicit awaiter(coroutine_handle<promise_type> h) noexcept
            : coro_(h)
        {
            print(PRI3, "%p: task::awaiter::awaiter(...)\n", this);
        }

        coroutine_handle<promise_type> coro_;
    };

    awaiter operator co_await() && noexcept {
        print(PRI3, "%p: task::operation co_await() -> awaiter\n", this);
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
