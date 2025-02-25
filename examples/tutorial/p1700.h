/**
 * @file p1700.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _P1700_H_
#define _P1700_H_

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

#include "use_mode.h"

using namespace corolib;

void start_operation_impl(UseMode useMode, async_operation<int>& op);

extern UseMode useMode;

async_ltask<int> coroutine1();
async_ltask<void> coroutine0();

void reinit();

#endif
