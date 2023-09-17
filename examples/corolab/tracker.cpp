/**
 *  Filename: tracker.cpp
 *  Description:
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)/
 */

#include "tracker.h"
#include "print.h"

tracker::tracker()
{
    //print("tracker::tracker\n");
}

tracker::~tracker()
{
    //print("tracker::~tracker\n");
    print("--------------------------------------------\n");
    print("\tcons\tdest\tdiff\tmax\n");
    print("cor\t%d\t%d\t%d\t%d\n",
        nr_coroutines_constructed,
        nr_coroutines_destructed,
        nr_coroutines_constructed - nr_coroutines_destructed,
        nr_max_simultaneously_present_coroutines);
    print("pro\t%d\t%d\t%d\t%d\n",
        nr_promise_types_constructed,
        nr_promise_types_destructed,
        nr_promise_types_constructed - nr_promise_types_destructed,
        nr_max_simultaneously_present_promise_types);
}

tracker tracker_obj;


promise_type_tracker::promise_type_tracker()
{
    ++tracker_obj.nr_promise_types_constructed;
    ++tracker_obj.nr_currently_present_promise_types;
    if (tracker_obj.nr_currently_present_promise_types > tracker_obj.nr_max_simultaneously_present_promise_types)
        tracker_obj.nr_max_simultaneously_present_promise_types++;
}

promise_type_tracker::~promise_type_tracker()
{
    ++tracker_obj.nr_promise_types_destructed;
    --tracker_obj.nr_currently_present_promise_types;
}


coroutine_tracker::coroutine_tracker()
{
    ++tracker_obj.nr_coroutines_constructed;
    ++tracker_obj.nr_currently_present_coroutines;
    if (tracker_obj.nr_currently_present_coroutines > tracker_obj.nr_max_simultaneously_present_coroutines)
        tracker_obj.nr_max_simultaneously_present_coroutines++;
}

coroutine_tracker::~coroutine_tracker()
{
    ++tracker_obj.nr_coroutines_destructed;
    --tracker_obj.nr_currently_present_coroutines;
}

