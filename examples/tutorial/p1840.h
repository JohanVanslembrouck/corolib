/**
 * @file p1840.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1840_H_
#define _P1840_H_

#include <optional>

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

extern async_operation<std::optional<int>> op1;
void start_operation_impl(async_operation<std::optional<int>>& op);

async_task<int> coroutine1();
async_task<int> coroutine2();
async_task<int> coroutine3();

#endif
