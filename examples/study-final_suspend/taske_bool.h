/**
 * @file taske_bool.h
 * @brief 
 *
 * Uses eager start.
 * 
 * task::promise_type::final_awaiter::await_suspend() returns void.
 * task::awaiter::await_suspend() returns bool.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _TASKE_BOOL_H_
#define _TASKE_BOOL_H_

#include <coroutine>
#include <atomic>

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
                auto& promise = h.promise();
                if (promise.ready.exchange(true, std::memory_order_acq_rel)) {
                    if (h.promise().continuation)
                        h.promise().continuation.resume();
                }
            }
            void await_resume() noexcept {}
        };

        final_awaiter final_suspend() noexcept {
            return {};
        }
        
        coroutine_handle<> continuation{ nullptr };
        std::atomic<bool> ready{ false };
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

        bool await_suspend(coroutine_handle<> continuation) noexcept {
            auto& promise = coro_.promise();

            // Store the continuation in the task's promise so that the final_suspend()
            // knows to resume this coroutine when the task completes.
            promise.continuation = continuation;

            // If the final_suspend() method has already set the 'ready'
            // flag to 'true' before we return here then we want to return
            // 'false' from await_supsend() to indicate that the coroutine
            // should be immediately resumed.
            return !promise.ready.exchange(true, std::memory_order_acq_rel);
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
