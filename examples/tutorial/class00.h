/**
 * @file class00.h
 * @brief
 * Defines a "class" with (the simulation of) one asynchronous operation.
 * This file does not use a real C++ class, but defines all functions at the global level.
 * See class01.h for the definition using a C++ class.
 * 
 * @author Johan Vanslembrouck
 */

#ifndef _CLASS00_H_
#define _CLASS00_H_

#include <functional>

#include <corolib/async_operation.h>
#include <corolib/threadawaker.h>

using namespace corolib;

#include "use_mode.h"

void async_op(std::function<void(int)>&& completionHandler);

void start_operation_impl(async_operation<int>& op);

extern UseMode useMode;

extern int delay;
extern ThreadAwaker* awaker;

#endif
