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

struct suspend_never_p {
    bool await_ready() const noexcept {
        print(PRI3, "%p: suspend_never_p::await_ready() -> bool: return true;\n", this);
        return true;
    }
    void await_suspend(coroutine_handle<>) const noexcept {
        print(PRI3, "%p: suspend_never_p::await_suspend(...) -> void\n", this);
    }
    void await_resume() const noexcept {
        print(PRI3, "%p: void suspend_never_p::await_resume() -> void\n", this);
    }
};

struct suspend_always_p {
    bool await_ready() const noexcept {
        print(PRI3, "%p: suspend_always_p::await_ready() -> bool: return false;\n", this);
        return false;
    }
    void await_suspend(coroutine_handle<>) const noexcept {
        print(PRI3, "%p: suspend_always_p::await_suspend(...) -> void\n", this);
    }
    void await_resume() const noexcept {
        print(PRI3, "%p: void suspend_always_p::await_resume() -> void\n", this);
    }
};

#endif
