/**
 * @file taske_bool.h
 * @brief 
 *
 * Uses eager start.
 * 
 * task::promise_type::final_awaiter::await_suspend() returns bool.
 * task::awaiter::await_suspend() returns bool.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _TASKE_BOOL_H_
#define _TASKE_BOOL_H_

#include <coroutine>
#include <atomic>

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

        suspend_never_p initial_suspend() noexcept {
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

        struct final_awaiter : public final_awaiter_tracker {
            bool await_ready() noexcept {
                print(PRI3, "%p: promise_type::final_awaiter::await_ready() -> bool: return false;\n", this);
                return false;
            }
            bool await_suspend(coroutine_handle<promise_type> h) noexcept {
                print(PRI3, "%p: promise_type::final_awaiter::await_suspend() -> bool: enter\n", this);
                auto& promise = h.promise();
                if (promise.ready.exchange(true, std::memory_order_acq_rel)) {
                    if (h.promise().continuation) {
                        print(PRI3, "%p: promise_type::final_awaiter::await_suspend() -> bool: before h.promise().continuation.resume();\n", this);
                        h.promise().continuation.resume();
                        print(PRI3, "%p: promise_type::final_awaiter::await_suspend() -> bool: after h.promise().continuation.resume();\n", this);
                    }
                }
                // This function must return true.
                // Otherwise the coroutine will run to an end and the coroutine frame will be deleted.
                // The call of get_result() will retrieve the result from released memory.
                print(PRI3, "%p: promise_type::final_awaiter::await_suspend() -> bool: leave: return true;\n", this);
                return true;
            }
            void await_resume() noexcept {
                print(PRI3, "%p: promise_type::final_awaiter::await_resume() -> void()\n", this);
            }
        };

        final_awaiter final_suspend() noexcept {
            print(PRI3, "%p: promise_type::final_suspend() -> final_awaiter()\n", this);
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

    class awaiter {
    public:
        bool await_ready() noexcept {
            int done = coro_.done();
            print(PRI3, "%p: task::awaiter::await_ready() -> bool: return %d;\n", this, done);
            return done;
        }

        bool await_suspend(coroutine_handle<> continuation) noexcept {
            print(PRI3, "%p: promise_type::final_awaiter::await_suspend() -> bool: enter\n", this);
            auto& promise = coro_.promise();

            // Store the continuation in the task's promise so that the final_suspend()
            // knows to resume this coroutine when the task completes.
            promise.continuation = continuation;

            // If the final_suspend() method has already set the 'ready'
            // flag to 'true' before we return here then we want to return
            // 'false' from await_supsend() to indicate that the coroutine
            // should be immediately resumed.
            bool ret = !promise.ready.exchange(true, std::memory_order_acq_rel);
            print(PRI3, "%p: promise_type::final_awaiter::await_suspend() -> bool: leave: return %d;\n", this, ret);
            return ret;
        }

        int await_resume() noexcept {
            print(PRI3, "%p: task::awaiter::await_resume() -> int\n", this);
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
