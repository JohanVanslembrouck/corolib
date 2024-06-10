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
#include <atomic>
#include <utility>

using namespace std;

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
                if (h.promise().continuation)
                    h.promise().continuation.resume();
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
    {}

    ~task() {
        if (coro_)
            if (coro_.done())
                coro_.destroy();
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
