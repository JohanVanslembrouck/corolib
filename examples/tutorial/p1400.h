/**
 * @file p1400.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#ifndef _P1400_H_
#define _P1400_H_

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>
#include <corolib/threadawaker.h>

#include "use_mode.h"

using namespace corolib;

void start_operation_impl(UseMode useMode, async_operation<int>* op);
extern int queueSize;
extern UseMode useMode;
async_task<int> coroutine1();
extern int delay;
extern ThreadAwaker *awaker;

#endif
