/**
 *  Filename: tracker.cpp
 *  Description:
 *
 *  Author: Johan Vanslembrouck
 */

#include <thread>

#include "corolib/tracker.h"
#include "corolib/print.h"

namespace corolib
{
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
        print(PRI1, "\tcons\tdest\tdiff\tmax\tc>p\tc<p\tc>h\tc<h\n");

        print(PRI1, "ope\t%d\t%d\t%d\t%d\n",
            nr_operations_constructed,                                      // cons
            nr_operations_destructed,                                       // dest
            nr_operations_constructed - nr_operations_destructed,           // diff
            nr_max_simultaneously_present_operations                        // max
        );
#if 0
        print(PRI1, "cor\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
            nr_coroutines_constructed,                                      // cons
            nr_coroutines_destructed,                                       // dest
            nr_coroutines_constructed - nr_coroutines_destructed,           // diff
            nr_max_simultaneously_present_coroutines,                       // max
            nr_dying_coroutines_detecting_dead_promise,                     // c>p
            nr_dying_coroutines_detecting_live_promise,
            nr_access_errors
        );
#endif
        print(PRI1, "cor\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
            nr_coroutines_constructed,                                      // cons
            nr_coroutines_destructed,                                       // dest
            nr_coroutines_constructed - nr_coroutines_destructed,           // diff
            nr_max_simultaneously_present_coroutines,                       // max
            nr_dying_coroutines_detecting_dead_promise,                     // c>p
            nr_dying_coroutines_detecting_live_promise,                     // c>p
            nr_dying_coroutines_handle_done,                                // c>h
            nr_dying_coroutines_handle_not_done                             // c<h
        );
        print(PRI1, "pro\t%d\t%d\t%d\t%d\t%d\t%d\n",
            nr_promise_types_constructed,                                   // cons
            nr_promise_types_destructed,                                    // dest
            nr_promise_types_constructed - nr_promise_types_destructed,     // diff
            nr_max_simultaneously_present_promise_types,                    // max
            nr_dying_promises_detecting_live_coroutine,                     // c>p
            nr_dying_promises_detecting_dead_coroutine                      // c<h
            //
            //
        );
        print(PRI1, "fin\t%d\t%d\t%d\t%d\t\t\t%d\t%d\n",
            nr_final_awaiters_constructed,                                  // cons
            nr_final_awaiters_destructed,                                   // dest
            nr_final_awaiters_constructed - nr_final_awaiters_destructed,   // diff
            nr_max_simultaneously_present_final_awaiters,                   // max
            //
            //
            nr_final_awaiters_await_suspend_returning_true,                 // c>h
            nr_final_awaiters_await_suspend_returning_false                 // c<h
        );
        print(PRI1, "---------------------------------------------------------------\n");
#if 0
        print(PRI1, "Legend:\n");
        print(PRI1, "  ope = # async_operation objects, cor = # async_task objects, pro = # async_task::promise_type objects\n");
        print(PRI1, "  fin = # async_task::promise_type::final_awaiter objects\n");
        print(PRI1, "  cons = # objects constructed, dest = # objects destructed, diff = cons - dest\n");
        print(PRI1, "  max = maximum # objects alive at any time\n");
        print(PRI1, "  c>p = # async_task objects with a longer  lifetime than the promise_type object in called coroutine\n");
        print(PRI1, "  c<p = # async_task objects with a shorter lifetime than the promise_type object in called coroutine\n");
//      print(PRI1, "  err = # attempts from a coroutine object to access the value in an invalid promise_type object\n");
        print(PRI1, "  c>h = # async_task objects with a longer  lifetime than the called coroutine (handle.done())\n");
        print(PRI1, "      = or: # final_awaiter::await_suspend() calls returning true\n");
        print(PRI1, "  c<h = # async_task objects with a shorter lifetime than the called coroutine (!handle.done())\n");
        print(PRI1, "      = or: # final_awaiter::await_suspend() calls returning false\n");
#endif
        print(PRI1, "Waiting %d milliseconds before exiting\n", m_delay);
        std::this_thread::sleep_for(std::chrono::milliseconds(m_delay));
    }

    tracker tracker_obj;
    int tracker::m_delay = 1000;

    /**
     * @brief promise_type_tracker
     *
     */

    promise_type_tracker::promise_type_tracker()
    {
        print(PRI3, "p: promise_type_tracker::~promise_type_tracker()\n", this);
        ++tracker_obj.nr_promise_types_constructed;
        ++tracker_obj.nr_currently_present_promise_types;
        if (tracker_obj.nr_currently_present_promise_types > tracker_obj.nr_max_simultaneously_present_promise_types)
            ++tracker_obj.nr_max_simultaneously_present_promise_types;
    }

    promise_type_tracker::~promise_type_tracker()
    {
        print(PRI3, "p: promise_type_tracker::~promise_type_tracker()\n", this);
        ++tracker_obj.nr_promise_types_destructed;
        --tracker_obj.nr_currently_present_promise_types;
    }

    /**
     * @brief coroutine_tracker
     *
     */

    coroutine_tracker::coroutine_tracker()
    {
        print(PRI3, "p: coroutine_tracker::~coroutine_tracker()\n", this);
        ++tracker_obj.nr_coroutines_constructed;
        ++tracker_obj.nr_currently_present_coroutines;
        if (tracker_obj.nr_currently_present_coroutines > tracker_obj.nr_max_simultaneously_present_coroutines)
            ++tracker_obj.nr_max_simultaneously_present_coroutines;
    }

    coroutine_tracker::~coroutine_tracker()
    {
        print(PRI3, "p: coroutine_tracker::~coroutine_tracker()\n", this);
        ++tracker_obj.nr_coroutines_destructed;
        --tracker_obj.nr_currently_present_coroutines;
    }

    /**
     * @brief operation_tracker
     *
     */

    operation_tracker::operation_tracker()
    {
        print(PRI3, "%p: operation_tracker::operation_tracker()\n", this);
        ++tracker_obj.nr_operations_constructed;
        ++tracker_obj.nr_currently_present_operations;
        if (tracker_obj.nr_currently_present_operations > tracker_obj.nr_max_simultaneously_present_operations)
            ++tracker_obj.nr_max_simultaneously_present_operations;
    }

    operation_tracker::~operation_tracker()
    {
        print(PRI3, "%p: operation_tracker::~operation_tracker()\n", this);
        ++tracker_obj.nr_operations_destructed;
        --tracker_obj.nr_currently_present_operations;
    }

    /**
     * @ brief final_awaiter_tracker
     * 
     */

    final_awaiter_tracker::final_awaiter_tracker()
    {
        print(PRI3, "p: final_awaiter_tracker::~final_awaiter_tracker()\n", this);
        ++tracker_obj.nr_final_awaiters_constructed;
        ++tracker_obj.nr_currently_present_final_awaiters;
        if (tracker_obj.nr_currently_present_final_awaiters > tracker_obj.nr_max_simultaneously_present_final_awaiters)
            ++tracker_obj.nr_max_simultaneously_present_final_awaiters;
    }

    final_awaiter_tracker::~final_awaiter_tracker()
    {
        print(PRI3, "p: final_awaiter_tracker::~final_awaiter_tracker()\n", this);
        ++tracker_obj.nr_final_awaiters_destructed;
        --tracker_obj.nr_currently_present_final_awaiters;
    }

}
