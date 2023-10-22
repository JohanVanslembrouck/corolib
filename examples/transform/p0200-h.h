/**
 *  Filename: p0200-h.h
 *  Description
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P0200_H_H_
#define _P0200_H_H_

#include "config.h"
#include "p0200.h"

task h(int x, int y);

__coroutine_state* __h_resume(__coroutine_state* s);
void __h_destroy(__coroutine_state* s);

using __h_promise_t = std::coroutine_traits<task, int, int>::promise_type;

/////
// The coroutine-state definition

struct __h_state : __coroutine_state_with_promise<__h_promise_t> {
    __h_state(int&& x, int&& y)
        : x(static_cast<int&&>(x))
        , y(static_cast<int&&>(y)) {
        // Initialise the function-pointers used by coroutine_handle::resume/destroy/done().
        this->__resume = &__h_resume;
        this->__destroy = &__h_destroy;

        // Use placement-new to initialise the promise object in the base-class
        // after we've initialised the argument copies.
        ::new ((void*)std::addressof(this->__promise))
            __h_promise_t(construct_promise<__h_promise_t>(this->x, this->y));
    }

    ~__h_state() {
        this->__promise.~__h_promise_t();
    }

    int __suspend_point = 0;

    // Argument copies
    int x;
    int y;

    // Local variables/temporaries
    struct __scope1 {
        manual_lifetime<task> __tmp2;
        manual_lifetime<task::awaiter> __tmp3;
    };
    union {
        manual_lifetime<std::suspend_never> __tmp1;
        __scope1 __s1;
        manual_lifetime<std::suspend_always> __tmp4;
    };
};

/////
// The "ramp" function

task h(int x, int y) {
    std::unique_ptr<__h_state> state(new __h_state(static_cast<int&&>(x), static_cast<int&&>(y)));
    decltype(auto) return_value = state->__promise.get_return_object();

    print(PRI3, "h(%d, %d): co_await initial_suspend();\n", x, y);
    state->__tmp1.construct_from([&]() -> decltype(auto) {
        return state->__promise.initial_suspend();
        });
    if (!state->__tmp1.get().await_ready()) {
        state->__tmp1.get().await_suspend(
            std::coroutine_handle<__g_promise_t>::from_promise(state->__promise));
        state.release();
        // fall through to return statement below.
    }
    else {
        // Coroutine did not suspend. Start executing the body immediately.
        __h_resume(state.release());
    }
    return return_value;
}

/////
//  The "resume" function

__coroutine_state* __h_resume(__coroutine_state* s) {
    auto* state = static_cast<__h_state*>(s);

    std::coroutine_handle<void> coro_to_resume;

    try {
        switch (state->__suspend_point) {
        case 0: goto suspend_point_0;
        case 1: goto suspend_point_1;
//      default: std::unreachable();       // 'unreachable': is not a member of 'std'     // JVS
        default:;
        }

    suspend_point_0:
        {
            destructor_guard tmp1_dtor{ state->__tmp1 };
            state->__tmp1.get().await_resume();
        }

        print(PRI1, "h(%d, %d): int i = co_await g(%d);\n", state->x, state->y, state->x);
        {
            state->__s1.__tmp2.construct_from([&] {
                return g(state->x);
                });
            destructor_guard tmp2_dtor{ state->__s1.__tmp2 };

            state->__s1.__tmp3.construct_from([&] {
                return static_cast<task&&>(state->__s1.__tmp2.get()).operator co_await();
                });
            destructor_guard tmp3_dtor{ state->__s1.__tmp3 };

            if (!state->__s1.__tmp3.get().await_ready()) {
                state->__suspend_point = 1;

                state->__s1.__tmp3.get().await_suspend(
                    std::coroutine_handle<__g_promise_t>::from_promise(state->__promise));

                // A coroutine suspends without exiting scopes - so cancel the destructor-guards.
                tmp3_dtor.cancel();
                tmp2_dtor.cancel();

                return static_cast<__coroutine_state*>(std::noop_coroutine().address());
            }

            // Don't exit the scope here.
            // We can't 'goto' a label that enters the scope of a variable with a non-trivial
            // destructor. So we have to exit the scope of the destructor guards here without
            // calling the destructors and then recreate them after the `suspend_point_1` label.
            tmp3_dtor.cancel();
            tmp2_dtor.cancel();
        }

    suspend_point_1:
        int i = [&]() -> decltype(auto) {
            destructor_guard tmp2_dtor{ state->__s1.__tmp2 };
            destructor_guard tmp3_dtor{ state->__s1.__tmp3 };
            return state->__s1.__tmp3.get().await_resume();
            }();

            print(PRI1, "h(%d, %d): co_return y + i (= %d);\n", state->x, state->y, state->y + i);
            state->__promise.return_value(state->y + i);
            goto final_suspend;
    }
    catch (...) {
        state->__promise.unhandled_exception();
        goto final_suspend;
    }

final_suspend:
    print(PRI3, "h(%d, %d): co_await promise.final_suspend();\n", state->x, state->y);
    {
        state->__tmp4.construct_from([&]() noexcept {
            return state->__promise.final_suspend();
            });
        destructor_guard tmp4_dtor{ state->__tmp4 };

        if (!state->__tmp4.get().await_ready()) {
            state->__suspend_point = 2;
            state->__resume = nullptr; // mark as final suspend-point

            state->__tmp4.get().await_suspend(
                std::coroutine_handle<__h_promise_t>::from_promise(state->__promise));

            tmp4_dtor.cancel();
            return static_cast<__coroutine_state*>(std::noop_coroutine().address());
        }

        state->__tmp4.get().await_resume();
    }

    //  Destroy coroutine-state if execution flows off end of coroutine
    delete state;

    return static_cast<__coroutine_state*>(std::noop_coroutine().address());
}

/////
// The "destroy" function

void __h_destroy(__coroutine_state* s) {
    auto* state = static_cast<__h_state*>(s);

    switch (state->__suspend_point) {
    case 0: goto suspend_point_0;
    case 1: goto suspend_point_1;
    case 2: goto suspend_point_2;
 // default: std::unreachable();       // 'unreachable': is not a member of 'std'     // JVS
    default:;
    }

suspend_point_0:
    state->__tmp1.destroy();
    goto destroy_state;

suspend_point_1:
    state->__s1.__tmp3.destroy();
    state->__s1.__tmp2.destroy();
    goto destroy_state;

suspend_point_2:
    state->__tmp4.destroy();
    goto destroy_state;

destroy_state:
    delete state;

}

#endif