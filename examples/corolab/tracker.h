/**
 *  Filename: tracker.h
 *  Description:
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)/
 */

#ifndef _TRACKER_H_
#define _TRACKER_H_

struct tracker
{
    tracker();
    ~tracker();

    int nr_promise_types_constructed = 0;
    int nr_promise_types_destructed = 0;
    int nr_currently_present_promise_types = 0;
    int nr_max_simultaneously_present_promise_types = 0;

    int nr_coroutines_constructed = 0;
    int nr_coroutines_destructed = 0;
    int nr_currently_present_coroutines = 0;
    int nr_max_simultaneously_present_coroutines = 0;
};

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

#endif