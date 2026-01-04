/**
 * @file task_bool2.h
 * @brief
 *
 * Contains the task class that is used in p0110_bool.cpp.
 * 
 * task::promise_type::final_awaiter::await_ready() returns true.
 * task::promise_type::final_awaiter::await_suspend() returns bool (true).
 * task::awaiter::await_suspend() returns bool.
 * 
 * A major difference with the other task files is that the task destructor in this file
 * has in essence an empty implementation: it will not call destroy() on any coroutine_handle.
 * The reason is that the coroutines using this task definition will run to completion
 * (i.e., they will not suspend at the final suspend point).
 * 
 * @author Johan Vanslembrouck
 */

#ifndef _TASK_BOOL2_H_
#define _TASK_BOOL2_H_

#include <coroutine>
#include <atomic>

#include "tracker.h"
#include "suspend.h"
#include "print.h"

using namespace std;

#define ALLOW_CO_AWAIT_TASK_OBJECT 1

class task : private coroutine_tracker {
public:
    class promise_type : private promise_type_tracker {
    public:
        promise_type() {
            print(PRI2, "%p: promise_type::promise_type()\n", this);
        }

        ~promise_type() {
            print(PRI2, "%p: promise_type::~promise_type()\n", this);
            mytask = nullptr;
        }

        task get_return_object() noexcept {
            print(PRI3, "%p: promise_type::get_return_object() -> task\n", this);
            return task{ coroutine_handle<promise_type>::from_promise(*this) };
        }

        suspend_always_p initial_suspend() noexcept {
            print(PRI3, "%p: promise_type::initial_suspend() -> suspend_always_p\n", this);
            return {};
        }

        void link_task(task* t) {
            mytask = t;
        }

        void return_value(int v) noexcept {
            print(PRI3, "%p: promise_type::return_value(%d) -> void\n", this, v);
            if (mytask)
                mytask->value = v;
        }

        void unhandled_exception() noexcept {
            print(PRI3, "%p: promise_type::unhandled_exception() -> void\n", this);
            std::terminate();
        }

        struct final_awaiter : private final_awaiter_tracker {
            promise_type* p{ nullptr };

            final_awaiter(promise_type* p1)
                : p(p1) {
                print(PRI3, "%p: promise_type::final_awaiter::final_awaiter(%p)\n", this, p);
            }
            ~final_awaiter() {
                print(PRI3, "%p: promise_type::final_awaiter::~final_awaiter()\n", this);
                p = nullptr;
            }
            bool await_ready() noexcept {
                print(PRI3, "%p: promise_type::final_awaiter::await_ready() -> bool: return true;\n", this);
                return true;
            }
            // The following function will never be called.
            bool await_suspend(coroutine_handle<promise_type> h) noexcept {
                print(PRI3, "%p: promise_type::final_awaiter::await_suspend() -> bool\n", this);
                return true;
            }
            void await_resume() noexcept {
                print(PRI3, "%p: promise_type::final_awaiter::await_resume() -> void(): enter\n", this);
                if (p)
                    if (p->continuation)
                        p->continuation.resume();
                    else
                        print(PRI3, "%p: promise_type::final_awaiter::await_resume() -> void(): nothing to resume\n", this);
                else
                    print(PRI3, "%p: promise_type::final_awaiter::await_resume() -> void(): nothing to resume (1)\n", this);
                print(PRI3, "%p: promise_type::final_awaiter::await_resume() -> void(): leave\n", this);
            }
        };

        final_awaiter final_suspend() noexcept {
            print(PRI3, "%p: promise_type::final_suspend() -> final_awaiter()\n", this);
            return {this};
        }

        coroutine_handle<> continuation{ nullptr };
        task* mytask{ nullptr };
        std::atomic<bool> ready{ false };
    };

#if 0
    task(task&& t) noexcept
        : coro_(std::exchange(t.coro_, {}))
    {}
#endif

    ~task() {
        print(PRI2, "%p: task::~task(): coro_.done() = %d\n", this, coro_.done());
    }

    int get_result() {
        print(PRI3, "%p: task::get_result() -> int: return %d;\n", this, value);
        return value;
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
            print(PRI3, "%p: task::awaiter::await_ready() -> bool: return false;\n", this);
            return false;
        }

        bool await_suspend(coroutine_handle<> continuation) noexcept {
            print(PRI3, "%p: task::awaiter::await_suspend() -> bool: enter\n", this);
            auto& promise = coro_.promise();

            // Store the continuation in the task's promise so that the final_suspend()
            // knows to resume this coroutine when the task completes.
            promise.continuation = continuation;

            // Then we resume the task's coroutine, which is currently suspended
            // at the initial-suspend-point (ie. at the open curly brace).
            if (coro_) {
                print(PRI3, "%p: task::awaiter::await_suspend() -> bool: before coro_.resume();\n", this);
                coro_.resume();
                print(PRI3, "%p: task::awaiter::await_suspend() -> bool: after coro_.resume();\n", this);
            }

            // If the final_suspend() method has already set the 'ready'
            // flag to 'true' before we return here then we want to return
            // 'false' from await_suspend() to indicate that the coroutine
            // should be immediately resumed.
            bool ret = !promise.ready.exchange(true, std::memory_order_acq_rel);
            print(PRI3, "%p: task::awaiter::await_suspend() -> bool: leave: return %d;\n", this, ret);
            return ret;
        }

        int await_resume() noexcept {
            print(PRI3, "%p: task::awaiter::await_resume() -> int\n", this);
            return mytask_->value;
        }

    private:
        friend task;
        explicit awaiter(coroutine_handle<promise_type> h, task* t) noexcept
            : coro_(h)
            , mytask_(t)
        {
            print(PRI3, "%p: task::awaiter::awaiter(...)\n", this);
        }

        coroutine_handle<promise_type> coro_;
        task* mytask_;
    };

#if ALLOW_CO_AWAIT_TASK_OBJECT
    awaiter operator co_await() noexcept {
        print(PRI3, "%p: task::operation co_await() -> awaiter\n", this);
        return awaiter{ coro_, this };
    }
#else
    /*
     * task t = coroutineX();
     * int v = co_await t;      // Not possible with the && variant
     */
    awaiter operator co_await() && noexcept {
        print(PRI3, "%p: task::operation co_await() && -> awaiter\n", this);
        return awaiter{ coro_, this };
    }
#endif

private:
    explicit task(coroutine_handle<promise_type> h) noexcept
        : coro_(h)
    {
        print(PRI2, "%p: task::task(...)\n", this);
        coro_.promise().link_task(this);
    }

    coroutine_handle<promise_type> coro_{ };
    int value{ 0 };
};

#endif
