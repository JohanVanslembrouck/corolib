/**
 *  Filename: tracker.cpp
 *  Description:
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
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
    print("--------------------------------------------------------\n");
#if 0
    print("\tcons\tdest\tdiff\tmax\tc>p\tp>c\terr\n");
    print("cor\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
        nr_coroutines_constructed,
        nr_coroutines_destructed,
        nr_coroutines_constructed - nr_coroutines_destructed,
        nr_max_simultaneously_present_coroutines,
        nr_dying_coroutines_detecting_dead_promise,
        nr_dying_coroutines_detecting_live_promise,
        nr_access_errors
        );
    print("pro\t%d\t%d\t%d\t%d\t%d\t%d\n",
        nr_promise_types_constructed,
        nr_promise_types_destructed,
        nr_promise_types_constructed - nr_promise_types_destructed,
        nr_max_simultaneously_present_promise_types,
        nr_dying_promises_detecting_live_coroutine,
        nr_dying_promises_detecting_dead_coroutine);
    print("fin\t%d\t%d\t%d\t%d\t%d\t%d\n",
        nr_final_awaiters_constructed,
        nr_final_awaiters_destructed,
        nr_final_awaiters_constructed - nr_final_awaiters_destructed,
        nr_max_simultaneously_present_final_awaiters,
        nr_final_awaiters_await_suspend_returning_true,
        nr_final_awaiters_await_suspend_returning_false);
#else
    print("\tcons\tdest\tdiff\tmax\n");
    print("cor\t%d\t%d\t%d\t%d\n",
        nr_coroutines_constructed,
        nr_coroutines_destructed,
        nr_coroutines_constructed - nr_coroutines_destructed,
        nr_max_simultaneously_present_coroutines
    );
    print("pro\t%d\t%d\t%d\t%d\n",
        nr_promise_types_constructed,
        nr_promise_types_destructed,
        nr_promise_types_constructed - nr_promise_types_destructed,
        nr_max_simultaneously_present_promise_types);
    print("fin\t%d\t%d\t%d\t%d\n",
        nr_final_awaiters_constructed,
        nr_final_awaiters_destructed,
        nr_final_awaiters_constructed - nr_final_awaiters_destructed,
        nr_max_simultaneously_present_final_awaiters);
#endif
    print("--------------------------------------------------------\n");
    print(PRI1, "Waiting 1000 milliseconds before exiting\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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


final_awaiter_tracker::final_awaiter_tracker()
{
    ++tracker_obj.nr_final_awaiters_constructed;
    ++tracker_obj.nr_currently_present_final_awaiters;
    if (tracker_obj.nr_currently_present_final_awaiters > tracker_obj.nr_max_simultaneously_present_final_awaiters)
        tracker_obj.nr_max_simultaneously_present_final_awaiters++;
}

final_awaiter_tracker::~final_awaiter_tracker()
{
    ++tracker_obj.nr_final_awaiters_destructed;
    --tracker_obj.nr_currently_present_final_awaiters;
}

