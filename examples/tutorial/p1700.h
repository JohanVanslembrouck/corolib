/**
 * @file p1700.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#ifndef _P1700_H_
#define _P1700_H_

#include <corolib/async_task.h>

using namespace corolib;

#include "use_mode.h"

extern UseMode useMode;

async_ltask<int> coroutine1();
async_ltask<void> coroutine0();

void reinit();

#endif
