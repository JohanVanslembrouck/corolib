/**
 *  Filename: p0302-f.h
 *  Description:
 *  This file contains the manual transformation of
 *
 *  task f(int x) {
 *      print(PRI1, "f(%d): co_return 42 + x (= %d);\n", x, 42 + x);
 *      co_return 42 + x;
 *  }
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P0302_F_H_
#define _P0302_F_H_

#include "config.h"
#include "print.h"

//#include "p0300.h"

#define USE_IMPLEMENTATION_FROM_PANICSOFTWARE_BLOG 0

task f(int x);

__coroutine_state* __f_resume(__coroutine_state* s);
void __f_destroy(__coroutine_state* s);

using __f_promise_t = std::coroutine_traits<task, int>::promise_type;


/////
// The coroutine-state definition

struct __f_state : __coroutine_state_with_promise<__f_promise_t> {
    __f_state(int&& x)
        : x(static_cast<int&&>(x)) {
        // Initialise the function-pointers used by coroutine_handle::resume/destroy/done().
        this->__resume = &__f_resume;
        this->__destroy = &__f_destroy;

        // Use placement-new to initialise the promise object in the base-class
        // after we've initialised the argument copies.
        ::new ((void*)std::addressof(this->__promise))
            __f_promise_t(construct_promise<__f_promise_t>(this->x));
    }

    ~__f_state() {
        this->__promise.~__f_promise_t();
    }

    int __suspend_point = 0;

    // Argument copies
    int x;

    union {
        manual_lifetime<std::suspend_never> __tmp1;
#if USE_FINAL_AWAITER
        manual_lifetime<task::promise_type::final_awaiter> __tmp2;
#else
        manual_lifetime<std::suspend_always> __tmp2;
#endif
    };
};

/////
// The "ramp" function

task f(int x) {
    std::unique_ptr<__f_state> state(new __f_state(static_cast<int&&>(x)));
    decltype(auto) return_value = state->__promise.get_return_object();

    print(PRI4, "f(%d): co_await promise.initial_suspend();\n", x);
    state->__tmp1.construct_from([&]() -> decltype(auto) {
        return state->__promise.initial_suspend();
        });
    if (!state->__tmp1.get().await_ready()) {
        state->__tmp1.get().await_suspend(
            std::coroutine_handle<__f_promise_t>::from_promise(state->__promise));
        state.release();
        // fall through to return statement below.
    }
    else {
        // Coroutine did not suspend. Start executing the body immediately.
        __f_resume(state.release());
    }
    return return_value;
}

/////
//  The "resume" function

__coroutine_state* __f_resume(__coroutine_state* s) {
    auto* state = static_cast<__f_state*>(s);

    std::coroutine_handle<void> coro_to_resume;

    try {
        switch (state->__suspend_point) {
        case 0: goto suspend_point_0;
        //default: std::unreachable();       // 'unreachable': is not a member of 'std'     // JVS
        default:;
        }

 suspend_point_0:
        {
            destructor_guard tmp1_dtor{ state->__tmp1 };
            state->__tmp1.get().await_resume();
        }

        print(PRI1, "f(%d): co_return 42 + x (= %d);\n", state->x, 42 + state->x);
        state->__promise.return_value(42 + state->x);
        goto final_suspend;
    }
    catch (...) {
        state->__promise.unhandled_exception();
        goto final_suspend;
    }

final_suspend:
    print(PRI4, "f(%d): co_await promise.final_suspend();\n", state->x);
    {
        state->__tmp2.construct_from([&]() noexcept {
            return state->__promise.final_suspend();
            });
        destructor_guard tmp2_dtor{ state->__tmp2 };
#if FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_VOID
        if (!state->__tmp2.get().await_ready()) {
            state->__suspend_point = 2;
            state->__resume = nullptr; // mark as final suspend-point

            state->__tmp2.get().await_suspend(
                std::coroutine_handle<__f_promise_t>::from_promise(state->__promise));

            tmp2_dtor.cancel();
            return static_cast<__coroutine_state*>(std::noop_coroutine().address());
        }
#endif
#if FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL
        bool await_suspend_result = false;
        if (!state->__tmp2.get().await_ready()) {
            state->__suspend_point = 2;
            state->__resume = nullptr; // mark as final suspend-point

            await_suspend_result = state->__tmp2.get().await_suspend(
                std::coroutine_handle<__f_promise_t>::from_promise(state->__promise));

            tmp2_dtor.cancel();

            print(PRI1, "f(%d): await_suspend_result = %d\n", state->x, await_suspend_result);
            if (await_suspend_result)
                return static_cast<__coroutine_state*>(std::noop_coroutine().address());
        }
#endif
#if FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE
        if (!state->__tmp2.get().await_ready()) {
            state->__suspend_point = 2;
            state->__resume = nullptr; // mark as final suspend-point

            std::coroutine_handle<> h = state->__tmp2.get().await_suspend(
                std::coroutine_handle<__f_promise_t>::from_promise(state->__promise));
#if USE_IMPLEMENTATION_FROM_PANICSOFTWARE_BLOG
            h.resume();
#endif
            tmp2_dtor.cancel();

#if USE_IMPLEMENTATION_FROM_PANICSOFTWARE_BLOG
            return static_cast<__coroutine_state*>(std::noop_coroutine().address());
#else
            return static_cast<__coroutine_state*>(h.address());
#endif
        }
#endif
        state->__tmp2.get().await_resume();
    }

    //  Destroy coroutine-state if execution flows off end of coroutine
    delete state;

    return static_cast<__coroutine_state*>(std::noop_coroutine().address());
}

/////
// The "destroy" function

void __f_destroy(__coroutine_state* s) {
    auto* state = static_cast<__f_state*>(s);

    switch (state->__suspend_point) {
    case 0: goto suspend_point_0;
    case 2: goto suspend_point_2;
    //default: std::unreachable();       // 'unreachable': is not a member of 'std'     // JVS
    default:;  // JVS
    }

suspend_point_0:
    state->__tmp1.destroy();
    goto destroy_state;

suspend_point_1:
    goto destroy_state;

suspend_point_2:
    state->__tmp2.destroy();
    goto destroy_state;

destroy_state:
    delete state;
}

#endif
