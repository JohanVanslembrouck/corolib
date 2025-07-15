/**
 * @file async_base.h
 * @brief async_base is the base class for async_operation and async_task classes.
 * It has been introduced to allow defining wait_all and wait_any in terms
 * of this class, instead of having to define them as class templates taking
 * an async_operation or async_task as template arguments.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */
#ifndef _ASYNC_BASE_H_
#define _ASYNC_BASE_H_

#include "print.h"
#include "when_all_counter.h"
#include "when_any_one.h"

namespace corolib
{
    class async_base
    {
    public:
        virtual bool is_ready() = 0;
        virtual void setCounter(when_all_counter* ctr) = 0;
        virtual void setWaitAny(when_any_one* waitany) = 0;
        virtual void start() = 0;
    };
}

#endif
