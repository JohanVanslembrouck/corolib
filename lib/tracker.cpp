/**
 *  Filename: tracker.cpp
 *  Description:
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)/
 */

#include <corolib/tracker.h>
#include <corolib/print.h>
#include <thread>

namespace corolib
{
    tracker::tracker()
    {
        //print(PRI1, "tracker::tracker\n");
    }

    tracker::~tracker()
    {
        //print(PRI1, "tracker::~tracker\n");
        print(PRI1, "--------------------------------------------------------\n");
        print(PRI1, "\tcons\tdest\tdiff\tmax\tc>p\tp>c\terr\n");
        print(PRI1, "ope\t%d\t%d\t%d\t%d\n",
            nr_operations_constructed,
            nr_operations_destructed,
            nr_operations_constructed - nr_operations_destructed,
            nr_max_simultaneously_present_operations);
        print(PRI1, "cor\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
            nr_coroutines_constructed,
            nr_coroutines_destructed,
            nr_coroutines_constructed - nr_coroutines_destructed,
            nr_max_simultaneously_present_coroutines,
            nr_dying_coroutines_detecting_dead_promise,
            nr_dying_coroutines_detecting_live_promise,
            nr_access_errors
        );
        print(PRI1, "pro\t%d\t%d\t%d\t%d\t%d\t%d\n",
            nr_promise_types_constructed,
            nr_promise_types_destructed,
            nr_promise_types_constructed - nr_promise_types_destructed,
            nr_max_simultaneously_present_promise_types,
            nr_dying_promises_detecting_live_coroutine,
            nr_dying_promises_detecting_dead_coroutine);
#if 1
        print(PRI1, "Legend:\n");
        print(PRI1, "  ope = operation objects, cor = coroutine objects, pro = promise_type objects\n");
        print(PRI1, "  cons = number of objects constructed, dest = number of objects destructed, diff = cons - dest\n");
        print(PRI1, "  max = maximum number of objects alive at any time\n");
        print(PRI1, "  c>p = number of coroutine objects with longer lifetime than the corresponding promise_type object\n");
        print(PRI1, "  p>c = number of promise_type objects with longer lifetime than the corresponding coroutine object\n");
        print(PRI1, "  err = number of attempts from a coroutine object to access the value in the promise_type object\n");
#endif
        print(PRI1, "Waiting 1 second before exiting\n");
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

    
    operation_tracker::operation_tracker()
    {
        ++tracker_obj.nr_operations_constructed;
        ++tracker_obj.nr_currently_present_operations;
        if (tracker_obj.nr_currently_present_operations > tracker_obj.nr_max_simultaneously_present_operations)
            tracker_obj.nr_max_simultaneously_present_operations++;
    }

    operation_tracker::~operation_tracker()
    {
        ++tracker_obj.nr_operations_destructed;
        --tracker_obj.nr_currently_present_coroutines;
    }
}
