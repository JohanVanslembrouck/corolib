/**
 * @file p1800.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#ifndef _P1800_H_
#define _P1800_H_

#include <corolib/async_task.h>
#include <corolib/async_operation.h>

#include "use_mode.h"

using namespace corolib;

extern UseMode useMode;

class Class1800
{
public:
    async_task<int> coroutine1();

    void start_operation_impl();

public:
    async_operation_rmc<int> op;
};

#endif
