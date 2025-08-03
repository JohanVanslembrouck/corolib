/**
 * @file p1800.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#ifndef _P1800_H_
#define _P1800_H_

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

#include "use_mode.h"

using namespace corolib;

extern UseMode useMode;
extern async_operation_rmc<int> op;
void start_operation_impl(async_operation_rmc<int>& op);
async_task<int> coroutine1();

#endif
