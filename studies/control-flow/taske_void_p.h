/**
 * @file taske_void_p.h
 * @brief
 * 
 * task::promise_type::initial_suspend() returns std::suspend_never.
 * task::promise_type::final_awaiter::await_suspend() returns void.
 * task::awaiter::await_suspend() returns void.
 * 
 * @author Johan Vanslembrouck
 */

#ifndef _TASKE_VOID_P_H_
#define _TASKE_VOID_P_H_

#include <stdio.h>
#include <coroutine>
#include <utility>

#include "print.h"
#include "counter.h"
#include "suspend.h"
#include "tracker.h"

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

        initial_awaiter_eager initial_suspend() noexcept {
            print(PRI3, "%p: promise_type::initial_suspend() -> suspend_never_p\n", this);
            return {};
        }

        void return_value(int v) noexcept {
            print(PRI3, "%p: promise_type::return_value(%d) -> void\n", this, v);
            value = v;
        }

        void unhandled_exception() noexcept {
            print(PRI3, "%p: promise_type::unhandled_exception() -> void\n", this);
            std::terminate();
        }

        struct final_awaiter : private final_awaiter_tracker {
            final_awaiter() {
                print(PRI3, "%p: promise_type::final_awaiter::final_awaiter()\n", this);
            }

            ~final_awaiter() {
                print(PRI3, "%p: promise_type::final_awaiter::~final_awaiter()\n", this);
            }

            bool await_ready() noexcept {
                counter++;
                print(PRI3, "%p: %d: promise_type::final_awaiter::await_ready() -> bool: return false\n", this, counter);
                return false;
            }

            void await_suspend(coroutine_handle<promise_type> h) noexcept {
                counter++;
                int counter1 = counter;
                print(PRI3, "%p: %d: promise_type::final_awaiter::await_suspend() -> void: enter\n", this, counter);
                if (h.promise().continuation) {
                    tracker_obj.nr_resumptions++;
                    counter++;
                    int counter2 = counter;
                    print(PRI3, "%p: %d: promise_type::final_awaiter::await_suspend() -> void: before h.promise().continuation.resume();\n", this, counter);
                    h.promise().continuation.resume();
                    print(PRI3, "%p: %d: promise_type::final_awaiter::await_suspend() -> void: after h.promise().continuation.resume();\n", this, counter2);
                }
                print(PRI3, "%p: %d: promise_type::final_awaiter::await_suspend() -> void: leave\n", this, counter1);
            }

            void await_resume() noexcept {
                counter++;
                print(PRI3, "%p: %d: promise_type::final_awaiter::await_resume() -> void\n", this, counter);
            }
        };

        final_awaiter final_suspend() noexcept {
            print(PRI3, "%p: promise_type::final_suspend() -> final_awaiter\n", this);
            return {};
        }

        coroutine_handle<> continuation = nullptr;
        int value{ 0 };
    };

    task(task&& t) noexcept 
        : coro_(std::exchange(t.coro_, {}))
    {
        print(PRI3, "%p: task::task(task&& t)\n", this);
    }

    ~task() {
        print(PRI2, "%p: task::~task()\n", this);
        if (coro_)
            if (coro_.done()) {
                print(PRI3, "%p: task::~task(): coro_.destroy();\n", this);
                coro_.destroy();
                coro_ = { };
            }
            else {
                print(PRI1, "%p: task::~task(): !coro.done()\n", this);
            }
        else
            print(PRI1, "%p: task::~task(): coro_ == nullptr\n", this);
    }

    void start() {
        counter++;
        print(PRI2, "%p: %d: task::start() -> void\n", this, counter);
    }

    int get_result() {
        counter++;
        if (coro_) {
            int value = coro_.promise().value;
            print(PRI3, "%p: %d: task::get_result() -> int; return %d\n", this, counter, value);
            return value;
        }
        else {
            print(PRI3, "%p: %d: task::get_result() -> int: return -1\n", counter, this);
            return -1;
        }
    }

    class awaiter : private awaiter_tracker {
    public:
        bool await_ready() noexcept {
            counter++;
            bool done = coro_.done();
            print(PRI3, "%p: %d: task::awaiter::await_ready() -> bool: return %d\n", this, counter, done);
            return done;
        }

        void await_suspend(coroutine_handle<> continuation) noexcept {
            counter++;
            print(PRI3, "%p: %d: task::awaiter::await_suspend() -> void: enter\n", this, counter);
            // Store the continuation in the task's promise so that the final_suspend()
            // knows to resume this coroutine when the task completes.
            coro_.promise().continuation = continuation;
            print(PRI3, "%p: %d: task::awaiter::await_suspend() -> void: leave\n", this, counter);
        }

        int await_resume() noexcept {
            counter++;
            int value = coro_.promise().value;
            print(PRI3, "%p: %d: task::awaiter::await_resume() -> int: return %d\n", this, counter, value);
            return value;
        }

        ~awaiter() {
            print(PRI3, "%p: task::awaiter::~awaiter(...)\n", this);
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

    /**
     * operator co_await was originally declared as
     *      awaiter operator co_await() && noexcept
     * This declaration does not allow to rewrite
     *    co_await foo();
     * as
     *    task f = foo();
     *    co_await f;
     */
    awaiter operator co_await() noexcept {
        print(PRI3, "%p: task::operation co_await() -> awaiter\n", this);
        return awaiter{ coro_ };
    }

private:
    explicit task(coroutine_handle<promise_type> h) noexcept
        : coro_(h)
    {
        print(PRI3, "%p: task::task(...)\n", this);
    }

    coroutine_handle<promise_type> coro_;
};

#endif
