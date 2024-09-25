/**
 * @file sync_wait_task.h
 * @brief
 *
 * Contains the sync_wait_task class that is common to p0000_void.cpp, p0110_void.cpp,
 * and p0110_bool.cpp, p0120_coroutine_handle.cpp.
 *
 * @author Lewis Baker
 */

#ifndef _SYNC_WAIT_TASK_H_
#define _SYNC_WAIT_TASK_H_

#include <coroutine>

using namespace std;

struct sync_wait_task {
    struct promise_type {
        sync_wait_task get_return_object() noexcept {
            return sync_wait_task{ coroutine_handle<promise_type>::from_promise(*this) };
        }

        suspend_never initial_suspend() noexcept { return{}; }

        suspend_always final_suspend() noexcept { return{}; }

        void return_void() noexcept {}

        void unhandled_exception() noexcept { std::terminate(); }
    };

    coroutine_handle<promise_type> coro_;

    explicit sync_wait_task(coroutine_handle<promise_type> h) noexcept : coro_(h) {}

    sync_wait_task(sync_wait_task&& t) noexcept : coro_(t.coro_) {
        t.coro_ = {};
    }

    ~sync_wait_task() {
        if (coro_) {
            coro_.destroy();
        }
    }

    static sync_wait_task start(task&& t) {
        co_await std::move(t);
    }

    bool done() {
        return coro_.done();
    }
};

#endif
