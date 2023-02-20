/**
 * @file p1400.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1400_H_
#define _P1400_H_

#include <functional>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

enum UseMode
{
    USE_NONE,
    USE_EVENTQUEUE,
    USE_THREAD,
    USE_THREAD_QUEUE,
    USE_IMMEDIATE_COMPLETION
};

void start_operation_impl(UseMode useMode, async_operation<int>* op);

extern int queueSize;

async_task<int> coroutine1();

#endif
