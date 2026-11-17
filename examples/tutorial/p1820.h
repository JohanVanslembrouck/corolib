/**
 * @file p1820.h
 * @brief
 *
 * 
 * @author Johan Vanslembrouck
 */

#ifndef _P1820_H_
#define _P1820_H_

#include <optional>

#include <corolib/async_task.h>
#include <corolib/async_operation.h>

#include "use_mode.h"

using namespace corolib;

extern UseMode useMode;

class Class1820
{
public:
    async_task<int> coroutine1();

    void start_operation_impl();

public:
    async_operation<std::optional<int>> op;
};

#endif
