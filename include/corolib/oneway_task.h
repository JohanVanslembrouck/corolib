/**
 * @file oneway_task.h
 * @brief
 * Implements in essence only a promise_type whose initial_suspend() and final_suspend()
 * both return a suspend_never object.
 * A oneway_task object cannot be co_awaited for.
 * Based upon oneway_task in cppcoro/include/cppcoro/async_scope.hpp.
 * 
 * @author Johan Vanslembrouck
 */

#ifndef _ONEWAY_TASK_H_
#define _ONEWAY_TASK_H_

#include <coroutine>

#include "print.h"

namespace corolib
{
    class oneway_task
    {
    public:
	
        oneway_task()
        {
            clprint(PRI2, "%p: oneway_task::oneway_task()\n", this);
        }

        ~oneway_task()
        {
            clprint(PRI2, "%p: oneway_task::~oneway_task()\n", this);
        }

        struct promise_type
        {
            promise_type()
            {
                clprint(PRI2, "%p: oneway_task::promise_type::promise_type()\n", this);
            }

            ~promise_type()
            {
                clprint(PRI2, "%p: oneway_task::promise_type::~promise_type()\n", this);
            }

            std::suspend_never initial_suspend() {
                clprint(PRI2, "%p: oneway_task::promise_type::initial_suspend()\n", this);
                return {};
            }

            std::suspend_never final_suspend() noexcept {
                clprint(PRI2, "%p: oneway_task::promise_type::final_suspend()\n", this);
                return {};
            }

            void unhandled_exception() {
                clprint(PRI2, "%p: oneway_task::promise_type::unhandled_exception()\n", this);
                std::terminate();
            }

            oneway_task get_return_object() {
                clprint(PRI2, "%p: oneway_task::promise_type::get_return_object()\n", this);
                return {};
            }

            void return_void() {
                clprint(PRI2, "%p: oneway_task::promise_type::return_void()\n", this);
            }
        };
    };
}

#endif