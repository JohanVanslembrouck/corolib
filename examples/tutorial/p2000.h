/**
 * @file p2000.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#ifndef _P2000_H_
#define _P2000_H_

#include <optional>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

#include "use_mode.h"

using namespace corolib;

extern UseMode useMode;
extern async_operation<std::optional<int>> op1;
void start_operation_impl(async_operation<std::optional<int>>& op);

void task1();
void task2();
void task3();

#endif
