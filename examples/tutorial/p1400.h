/**
 * @file p1400.h
 * @brief
 *
 * 
 * @author Johan Vanslembrouck
 */

#ifndef _P1400_H_
#define _P1400_H_

#include <corolib/async_task.h>

#include "use_mode.h"

using namespace corolib;

extern UseMode useMode;

async_task<int> coroutine1();

#endif
