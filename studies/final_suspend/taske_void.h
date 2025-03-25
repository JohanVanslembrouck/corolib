/**
 * @file taske_void.h
 * @brief
 * 
 * Uses eager start.
 *
 * task::promise_type::final_awaiter::await_suspend() returns void.
 * task::awaiter::await_suspend() returns void.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _TASKE_VOID_H_
#define _TASKE_VOID_H_

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

        suspend_never initial_suspend() noexcept {
            return {};
        }

        void return_value(int v) noexcept {
            value = v;
        }

        void unhandled_exception() noexcept {
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
#if USE_CORO_DONE_TEST
    ~task() {
        print(PRI2, "%p: task::~task(): test on coro_.done()\n", this);
        if (coro_)
            if (coro_.done()) {
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
            coro_.destroy();
            coro_ = {};
        }
        else
            print(PRI2, "%p: task::~task(): coro_ == nullptr\n", this);
    }
#endif

    int get_result() {
        if (coro_)
            return coro_.promise().value;
        else
            return -1;
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

        int await_resume() noexcept {
            if (coro_)
                return coro_.promise().value;
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
