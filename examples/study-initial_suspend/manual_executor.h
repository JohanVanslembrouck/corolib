/**
 * @file manual_executor.h
 * @brief
 *
 * Contains the manual_executor class that is common to p0000_void.cpp, p0110_void.cpp,
 * and p0110_bool.cpp, p0120_coroutine_handle.cpp.
 *
 * @author Lewis Baker
 */

#ifndef _MANUAL_EXECUTER_H_
#define _MANUAL_EXECUTER_H_

#include <coroutine>

using namespace std;

struct manual_executor {
    struct schedule_op {
        manual_executor& executor_;
        schedule_op* next_ = nullptr;
        coroutine_handle<> continuation_;

        schedule_op(manual_executor& executor)
            : executor_(executor)
        {}

        bool await_ready() noexcept { return false; }

        void await_suspend(coroutine_handle<> continuation) noexcept {
            continuation_ = continuation;
            next_ = executor_.head_;
            executor_.head_ = this;
        }

        void await_resume() noexcept {}
    };

    schedule_op* head_ = nullptr;

    schedule_op schedule() noexcept {
        return schedule_op{ *this };
    }

    void drain() {
        while (head_ != nullptr) {
            auto* item = head_;
            head_ = item->next_;
            item->continuation_.resume();
        }
    }

    void sync_wait(task&& t) {
        auto t2 = sync_wait_task::start(std::move(t));
        while (!t2.done()) {
            drain();
        }
    }
};

#endif
