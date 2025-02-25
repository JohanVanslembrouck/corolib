/**
 * @file p1820.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _P1820_H_
#define _P1820_H_

#include <optional>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

#include "use_mode.h"

using namespace corolib;

extern UseMode useMode;
extern async_operation<std::optional<int>> op;
void start_operation_impl(async_operation<std::optional<int>>& op);
async_task<int> coroutine1();

#endif
