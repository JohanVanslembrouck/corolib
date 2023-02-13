/**
 * @file p2110.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P2110_H_
#define _P2110_H_

#include "class01.h"

class CoroClass01
{
public:
    CoroClass01(Class01& object)
        : m_object(object)
    {
    }
    
    async_task<int> coroutine1();

protected:
    async_task<int> coroutine2();
    async_task<int> coroutine3();
    async_task<int> coroutine4();
    async_task<int> coroutine5();

public:
     Class01 m_object;
};

void completionflow(CoroClass01& coroObject);
int task1(CoroClass01&& coroObject);

#endif
