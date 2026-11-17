/**
 * @file p1830.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#ifndef _P1830_H_
#define _P1830_H_

#include <optional>

#include <corolib/async_task.h>
#include <corolib/async_operation.h>

#include "use_mode.h"

using namespace corolib;

extern UseMode useMode;

class Class1830
{
public:
    async_task<int> coroutine1();
  
    void start_operation_impl();

protected:
    async_task<int> coroutine2();
    async_task<int> coroutine3();
    async_task<int> coroutine4();

public:
    async_operation<std::optional<int>> op;

private:
    async_operation<std::optional<int>> op2;
    async_operation<std::optional<int>> op3;
    async_operation<std::optional<int>> op4;
};

#endif
