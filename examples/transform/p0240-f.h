/**
 *  Filename: p0240-f.h
 *  Description
 * 
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P0240_F_H_
#define _P0240_F_H_

#include "config.h"
#include "p0240.h"

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

    print(PRI4, "f(%d): co_await initial_suspend();\n", x);
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
        case 1: goto suspend_point_1;
 //     default: std::unreachable();       // 'unreachable': is not a member of 'std'     // JVS
        default:;
        }

    suspend_point_0:
        {
            destructor_guard tmp1_dtor{ state->__tmp1 };
            state->__tmp1.get().await_resume();
        }

        print(PRI1, "f(%d): co_await are1;\n", state->x);
        {
            auto_reset_event::awaiter aw = are1.operator co_await();

            if (!aw.await_ready()) {
                state->__suspend_point = 1;

                aw.await_suspend(
                    std::coroutine_handle<__f_promise_t>::from_promise(state->__promise));

                return static_cast<__coroutine_state*>(std::noop_coroutine().address());
            }
        }

    suspend_point_1:
        auto_reset_event::awaiter aw = are1.operator co_await();
        aw.await_resume();

        print(PRI1, "f(%d): co_return 42 + i (= %d);\n", state->x, 42 + state->x);
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

        if (!state->__tmp2.get().await_ready()) {
            state->__suspend_point = 2;
            state->__resume = nullptr; // mark as final suspend-point

            state->__tmp2.get().await_suspend(
                std::coroutine_handle<__f_promise_t>::from_promise(state->__promise));

            tmp2_dtor.cancel();
            return static_cast<__coroutine_state*>(std::noop_coroutine().address());
        }

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
    case 1: goto suspend_point_1;
    case 2: goto suspend_point_2;
//  default: std::unreachable();       // 'unreachable': is not a member of 'std'     // JVS
    default:;
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
