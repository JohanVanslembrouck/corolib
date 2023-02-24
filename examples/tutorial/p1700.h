/**
 * @file p1700.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1700_H_
#define _P1700_H_

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

enum class UseMode
{
    USE_NONE,
    USE_EVENTQUEUE,
    USE_THREAD,
    USE_THREAD_QUEUE,
    USE_IMMEDIATE_COMPLETION
};

extern UseMode useMode;
void start_operation_impl(UseMode useMode, async_operation<int>& op);
async_ltask<int> coroutine1();
async_ltask<void> coroutine0();

#endif
