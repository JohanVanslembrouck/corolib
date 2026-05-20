/**
 * @file async_base.h
 * @brief async_base is the base class for async_operation and async_task classes.
 * It has been introduced to allow defining wait_all and wait_any using async_base*.
 * This is the sole purpose of async_base: allow using async_operation and async_task
 * from when_all and when_any.
 * In a new approach using the WHEN_TYPE concept, it is possible to avoid
 * the use of this base class. This is the preferred approach.
 * See config.h on how to use the original and new configuration.
 *
 * @author Johan Vanslembrouck
 */

#ifndef _ASYNC_BASE_H_
#define _ASYNC_BASE_H_

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
