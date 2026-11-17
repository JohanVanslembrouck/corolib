/**
 * @file p1810.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#ifndef _P1810_H_
#define _P1810_H_

#include <corolib/async_task.h>
#include <corolib/async_operation.h>

#include "use_mode.h"

using namespace corolib;

extern UseMode useMode;

class Class1810
{
public:
    async_task<int> coroutine1();
   
    void start_operation_impl();

protected:
    async_task<int> coroutine1a();
    async_task<int> coroutine1b();

public:
    async_operation_rmc<int> op;
};

#endif
