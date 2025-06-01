/**
 *  Filename: tracker.h
 *  Description:
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _TRACKER_H_
#define _TRACKER_H_

struct tracker
{
    tracker();
    ~tracker();

    // promise_type related
    int nr_promise_types_constructed = 0;
    int nr_promise_types_destructed = 0;
    int nr_currently_present_promise_types = 0;
    int nr_max_simultaneously_present_promise_types = 0;

    int nr_dying_promises_detecting_live_coroutine = 0;
    int nr_dying_promises_detecting_dead_coroutine = 0;

    // coroutine related
    int nr_coroutines_constructed = 0;
    int nr_coroutines_destructed = 0;
    int nr_currently_present_coroutines = 0;
    int nr_max_simultaneously_present_coroutines = 0;

    int nr_dying_coroutines_detecting_live_promise = 0;
    int nr_dying_coroutines_detecting_dead_promise = 0;

    // final_awaiter related
    int nr_final_awaiters_constructed = 0;
    int nr_final_awaiters_destructed = 0;
    int nr_currently_present_final_awaiters = 0;
    int nr_max_simultaneously_present_final_awaiters = 0;

    int nr_final_awaiters_await_suspend_returning_true = 0;
    int nr_final_awaiters_await_suspend_returning_false = 0;

    int nr_access_errors = 0;
};

extern tracker tracker_obj;

struct promise_type_tracker
{
    promise_type_tracker();
    ~promise_type_tracker();
};

struct coroutine_tracker
{
    coroutine_tracker();
    ~coroutine_tracker();
};

struct final_awaiter_tracker
{
    final_awaiter_tracker();
    ~final_awaiter_tracker();
};

#endif