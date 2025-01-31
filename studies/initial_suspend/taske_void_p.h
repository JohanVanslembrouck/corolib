/**
 * @file taske_void_p.h
 * @brief
 * 
 * Uses eager start.
 *
 * task::promise_type::final_awaiter::await_suspend() returns void.
 * task::awaiter::await_suspend() returns void.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _TASKE_VOID_P_H_
#define _TASKE_VOID_P_H_

#include <stdio.h>
#include <coroutine>
#include <utility>

#include "print.h"
#include "tracker1.h"

using namespace std;

struct suspend_never_p {
    bool await_ready() const noexcept {
        print(PRI3, "%p: suspend_never::await_ready()\n", this);
        return true;
    }

    void await_suspend(coroutine_handle<>) const noexcept {
        print(PRI3, "%p: suspend_never::await_suspend(...)\n", this);
    }

    void await_resume() const noexcept {
        print(PRI3, "%p: suspend_never::await_resume()\n", this);
    }
};

class task {
public:
    class promise_type {
    public:
        promise_type() {
            print(PRI2, "%p: promise_type::promise_type()\n", this);
        }

        ~promise_type() {
            print(PRI2, "%p: promise_type::~promise_type()\n", this);
        }

        task get_return_object() noexcept {
            print(PRI3, "%p: promise_type::get_return_object()\n", this);
            return task{ coroutine_handle<promise_type>::from_promise(*this) };
        }

        suspend_never_p initial_suspend() noexcept {
            print(PRI3, "%p: promise_type::initial_suspend()\n", this);
            return {};
        }

        void return_value(int v) noexcept {
            print(PRI3, "%p: promise_type::return_value(%d)\n", this, v);
            value = v;
        }

        void unhandled_exception() noexcept {
            print(PRI3, "%p: promise_type::unhandled_exception()\n", this);
            std::terminate();
        }

        struct final_awaiter {
            final_awaiter() {
                print(PRI3, "%p: promise_type::final_awaiter::final_awaiter()\n", this);
            }

            ~final_awaiter() {
                print(PRI3, "%p: promise_type::final_awaiter::~final_awaiter()\n", this);
            }

            bool await_ready() noexcept {
                print(PRI3, "%p: promise_type::final_awaiter::await_ready()\n", this);
                return false;
            }

            void await_suspend(coroutine_handle<promise_type> h) noexcept {
                print(PRI3, "%p: promise_type::final_awaiter::await_suspend(): enter\n", this);
                if (h.promise().continuation) {
                    tracker1_obj.nr_resumptions++;
                    print(PRI3, "%p: promise_type::final_awaiter::await_suspend(): before h.promise().continuation.resume();\n", this);
                    h.promise().continuation.resume();
                    print(PRI3, "%p: promise_type::final_awaiter::await_suspend(): after  h.promise().continuation.resume();\n", this);
                }
                print(PRI3, "%p: promise_type::final_awaiter::await_suspend(): leave\n", this);
            }

            void await_resume() noexcept {
                print(PRI3, "%p: promise_type::final_awaiter::await_resume()\n", this);
            }
        };

        final_awaiter final_suspend() noexcept {
            print(PRI3, "%p: promise_type::final_suspend()\n", this);
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
        print(PRI2, "%p: task::start()\n", this);
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
            print(PRI3, "%p: task::awaiter::await_ready()\n", this);
            return coro_.done();
        }

        void await_suspend(coroutine_handle<> continuation) noexcept {
            print(PRI3, "%p: task::awaiter::await_suspend(): enter\n", this);
            // Store the continuation in the task's promise so that the final_suspend()
            // knows to resume this coroutine when the task completes.
            coro_.promise().continuation = continuation;
            print(PRI3, "%p: task::awaiter::await_suspend(): leave\n", this);
        }

        int await_resume() noexcept {
            print(PRI3, "%p: task::awaiter::await_resume()\n", this);
            return coro_.promise().value;
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
        print(PRI3, "%p: task::operation co_await()\n", this);
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
