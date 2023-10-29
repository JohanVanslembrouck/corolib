/**
 *  Filename: tracker.cpp
 *  Description:
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)/
 */

#include "tracker.h"
#include "print.h"

 /**
   * @brief tracker
   *
   */

tracker::tracker()
{
    //print(PRI1, "tracker::tracker\n");
}

tracker::~tracker()
{
    //print(PRI1, "tracker::~tracker\n");
    print(PRI1, "---------------------------------------------------------------\n");
    //print(PRI1, "\tcons\tdest\tdiff\tmax\tc>p\tp>c\terr\n");
#if 0
    print(PRI1, "\tcons\tdest\tdiff\tmax\tc>p\tc<p\tc>h\tc<h\n");
    print(PRI1, "cor\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
        nr_coroutines_constructed,
        nr_coroutines_destructed,
        nr_coroutines_constructed - nr_coroutines_destructed,
        nr_max_simultaneously_present_coroutines,
        nr_dying_coroutines_detecting_dead_promise,
        nr_dying_coroutines_detecting_live_promise,
        nr_access_errors
    );
#endif
#if 0
    print(PRI1, "cor\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
        nr_coroutines_constructed,
        nr_coroutines_destructed,
        nr_coroutines_constructed - nr_coroutines_destructed,
        nr_max_simultaneously_present_coroutines,
        nr_dying_coroutines_detecting_dead_promise,
        nr_dying_coroutines_detecting_live_promise,
        nr_dying_coroutines_handle_done,
        nr_dying_coroutines_handle_not_done
    );
    print(PRI1, "pro\t%d\t%d\t%d\t%d\t%d\t%d\n",
        nr_promise_types_constructed,
        nr_promise_types_destructed,
        nr_promise_types_constructed - nr_promise_types_destructed,
        nr_max_simultaneously_present_promise_types,
        nr_dying_promises_detecting_live_coroutine,
        nr_dying_promises_detecting_dead_coroutine);
    print(PRI1, "fin\t%d\t%d\t%d\t%d\t\t\t%d\t%d\n",
        nr_final_awaiters_constructed,
        nr_final_awaiters_destructed,
        nr_final_awaiters_constructed - nr_final_awaiters_destructed,
        nr_max_simultaneously_present_final_awaiters,
        nr_final_awaiters_await_suspend_returning_true,
        nr_final_awaiters_await_suspend_returning_false);
#else
    print(PRI1, "\tcons\tdest\tdiff\tmax\n");
    print(PRI1, "cor\t%d\t%d\t%d\t%d\n",
        nr_coroutines_constructed,
        nr_coroutines_destructed,
        nr_coroutines_constructed - nr_coroutines_destructed,
        nr_max_simultaneously_present_coroutines
    );
    print(PRI1, "pro\t%d\t%d\t%d\t%d\n",
        nr_promise_types_constructed,
        nr_promise_types_destructed,
        nr_promise_types_constructed - nr_promise_types_destructed,
        nr_max_simultaneously_present_promise_types);
    print(PRI1, "fin\t%d\t%d\t%d\t%d\n",
        nr_final_awaiters_constructed,
        nr_final_awaiters_destructed,
        nr_final_awaiters_constructed - nr_final_awaiters_destructed,
        nr_max_simultaneously_present_final_awaiters);
#endif
    print(PRI1, "---------------------------------------------------------------\n");
#if 0
    print(PRI1, "Legend:\n");
    print(PRI1, "  ope = # operation objects, cor = # coroutine objects, pro = # promise_type objects, fin = # final_awaiter objects\n");
    print(PRI1, "  cons = # objects constructed, dest = # objects destructed, diff = cons - dest\n");
    print(PRI1, "  max = maximum # objects alive at any time\n");
    print(PRI1, "  c>p = # coroutine objects with a longer lifetime than its promise_type object\n");
    print(PRI1, "  c<p = # coroutine objects with a shorter lifetime than its promise_type object\n");
    //      print(PRI1, "  err = # attempts from a coroutine object to access the value in an invalid promise_type object\n");
    print(PRI1, "  c>h = # coroutine objects with a longer lifetime than the coroutine handle (handle.done())\n");
    print(PRI1, "      = or: # final_awaiter::await_suspend() calls returning true\n");
    print(PRI1, "  c<h = # coroutine objects with a shorter lifetime than the coroutine handle (!handle.done())\n");
    print(PRI1, "      = or: # final_awaiter::await_suspend() calls returning false\n");
#endif
    print(PRI1, "Waiting 1 second before exiting\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

tracker tracker_obj;


/**
 * @brief promise_type_tracker
 *
 */

promise_type_tracker::promise_type_tracker()
{
    print(PRI3, "%p: promise_type_tracker::promise_type_tracker()\n", this);
    ++tracker_obj.nr_promise_types_constructed;
    ++tracker_obj.nr_currently_present_promise_types;
    if (tracker_obj.nr_currently_present_promise_types > tracker_obj.nr_max_simultaneously_present_promise_types)
        ++tracker_obj.nr_max_simultaneously_present_promise_types;
}

promise_type_tracker::~promise_type_tracker()
{
    print(PRI3, "%p: promise_type_tracker::~promise_type_tracker()\n", this);
    ++tracker_obj.nr_promise_types_destructed;
    --tracker_obj.nr_currently_present_promise_types;
}

/**
 * @brief coroutine_tracker
 *
 */

coroutine_tracker::coroutine_tracker()
{
    print(PRI3, "%p: coroutine_tracker::coroutine_tracker()\n", this);
    ++tracker_obj.nr_coroutines_constructed;
    ++tracker_obj.nr_currently_present_coroutines;
    if (tracker_obj.nr_currently_present_coroutines > tracker_obj.nr_max_simultaneously_present_coroutines)
        ++tracker_obj.nr_max_simultaneously_present_coroutines;
}

coroutine_tracker::~coroutine_tracker()
{
    print(PRI3, "%p: coroutine_tracker::~coroutine_tracker()\n", this);
    ++tracker_obj.nr_coroutines_destructed;
    --tracker_obj.nr_currently_present_coroutines;
}

/**
 * @ brief final_awaiter_tracker
 *
 */

final_awaiter_tracker::final_awaiter_tracker()
{
    print(PRI3, "%p: final_awaiter_tracker::final_awaiter_tracker()\n", this);
    ++tracker_obj.nr_final_awaiters_constructed;
    ++tracker_obj.nr_currently_present_final_awaiters;
    if (tracker_obj.nr_currently_present_final_awaiters > tracker_obj.nr_max_simultaneously_present_final_awaiters)
        ++tracker_obj.nr_max_simultaneously_present_final_awaiters;
}

final_awaiter_tracker::~final_awaiter_tracker()
{
    print(PRI3, "%p: final_awaiter_tracker::~final_awaiter_tracker()\n", this);
    ++tracker_obj.nr_final_awaiters_destructed;
    --tracker_obj.nr_currently_present_final_awaiters;
}
