/**
 * @file suspend.h
 * @brief
 *
 * Redefines std::suspend_never and std::suspend_always to print tracing information.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _SUSPEND_H_
#define _SUSPEND_H_

#include "print.h"
#include "tracker.h"
#include "counter.h"

struct suspend_never_p {
    bool await_ready() const noexcept {
        counter++;
        print(PRI3, "%p: %d: suspend_never_p::await_ready() -> bool: return true;\n", this, counter);
        return true;
    }
    void await_suspend(coroutine_handle<>) const noexcept {
        counter++;
        print(PRI3, "%p: %d: suspend_never_p::await_suspend(...) -> void\n", this, counter);
    }
    void await_resume() const noexcept {
        counter++;
        print(PRI3, "%p: %d: void suspend_never_p::await_resume() -> void\n", this, counter);
    }
};

struct suspend_always_p {
    bool await_ready() const noexcept {
        counter++;
        print(PRI3, "%p: %d: suspend_always_p::await_ready() -> bool: return false;\n", this, counter);
        return false;
    }
    void await_suspend(coroutine_handle<>) const noexcept {
        counter++;
        print(PRI3, "%p: %d: suspend_always_p::await_suspend(...) -> void\n", this, counter);
    }
    void await_resume() const noexcept {
        counter++;
        print(PRI3, "%p: %d: void suspend_always_p::await_resume() -> void\n", this, counter);
    }
};


struct initial_awaiter_eager : public suspend_never_p, private initial_awaiter_tracker
{
    initial_awaiter_eager() {
        print(PRI3, "%p: initial_awaiter_eager::initial_awaiter_eager()\n", this);
    }

    ~initial_awaiter_eager() {
        print(PRI3, "%p: initial_awaiter_eager::~initial_awaiter_eager()\n", this);
    }
};

struct initial_awaiter_lazy : public suspend_always_p, private initial_awaiter_tracker
{
    initial_awaiter_lazy() {
        print(PRI3, "%p: initial_awaiter_lazy::initial_awaiter_lazy()\n", this);
    }

    ~initial_awaiter_lazy() {
        print(PRI3, "%p: initial_awaiter_lazy::~initial_awaiter_lazy()\n", this);
    }
};

#endif
