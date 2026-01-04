/**
 * @file taske_sn2.h
 * @brief
 * 
 * Uses eager start.
 *
 * task::promise_type::final_suspend() returns std::suspend_never.
 * task::awaiter::await_suspend() returns void.
 * 
 * In contrast to taske_sn.h, value is stored in the task object instead of the promise_type object.
 * 
 * @author Johan Vanslembrouck
 */

#ifndef _TASKE_SN2_H_
#define _TASKE_SN2_H_

#include <coroutine>

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
        }

        task get_return_object() noexcept {
            print(PRI3, "%p: promise_type::get_return_object() -> task\n", this);
            return task{ coroutine_handle<promise_type>::from_promise(*this) };
        }

        suspend_never_p initial_suspend() noexcept {
            print(PRI3, "%p: promise_type::initial_suspend() -> suspend_never_p\n", this);
            return {};
        }

        void link_task(task* t) {
            mytask = t;
        }

        void return_value(int v) noexcept {
            print(PRI3, "%p: promise_type::return_value(%d) -> void: enter\n", this, v);
            if (mytask)
                mytask->value = v;
            if (continuation) {
                print(PRI3, "%p: promise_type::return_value(%d) -> void: before continuation.resume();\n", this, v);
                continuation.resume();
                print(PRI3, "%p: promise_type::return_value(%d) -> void: after continuation.resume();\n", this, v);
            }
            print(PRI3, "%p: promise_type::return_value(%d) -> void: leave\n", this, v);
        }

        void unhandled_exception() noexcept {
            print(PRI3, "%p: promise_type::return_value() -> void\n", this);
            std::terminate();
        }

        final_awaiter_suspend_never_p final_suspend() noexcept {
            print(PRI3, "%p: promise_type::final_suspend() -> final_awaiter_suspend_never_p\n", this);
            return {};
        }

        coroutine_handle<> continuation{ };
        task* mytask{ nullptr };
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
            else
                print(PRI2, "%p: task::~task(): !coro.done()\n", this);
        else
            print(PRI2, "%p: task::~task(): coro_ == { }\n", this);
    }
#else
    ~task() {
        print(PRI2, "%p: task::~task(): no test on coro_.done()\n", this);
        if (coro_) {
            print(PRI2, "%p: task::~task(): note: coro_.done() = %d\n", this, coro_.done());
            print(PRI2, "%p: task::~task(): coro_.destroy();\n", this);
            coro_.destroy();
            coro_ = {};
        }
        else
            print(PRI2, "%p: task::~task(): coro_ == { }\n", this);
    }
#endif

    int get_result() {
        print(PRI3, "%p: task::get_result() -> int: return %d;\n", this, value);
        return value;
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
            print(PRI3, "%p: task::awaiter::await_suspend() -> void: leave\n", this);
        }

        int await_resume() noexcept {
            print(PRI3, "%p: task::awaiter::await_resume() -> int: return %d;\n", this, mytask_->value);
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
        print(PRI2, "task::task(...)\n");
        coro_.promise().link_task(this);
    }

    coroutine_handle<promise_type> coro_{ };
    int value{ 0 };
};

#endif
