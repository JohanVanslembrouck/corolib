/**
 * @file p1610.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1610_H_
#define _P1610_H_

#include "class02.h"

enum class OperationState
{
    not_started,
    running,
    completed
};

class Class1490
{
public:
    Class1490(Class02& object)
        : m_object(object)
    {
    }

    async_task<int> coroutine1(int i);

public:
    Class02& m_object;

private:
    OperationState op2_state = OperationState::not_started;
    //async_operation<int> m_op2;
    async_operation_rmc<int> m_op2;
    int m_v2 = 0;
};

#endif