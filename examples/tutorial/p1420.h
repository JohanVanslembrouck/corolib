/**
 * @file p1420.h
 * @brief
 * Example with 5 coroutines.
 * coroutineI (I = 1..4) co_awaits coroutineI+1.
 * coroutine3 calls coroutine4 twice.
 * coroutine5 starts an asynchronous operation on object01 and on object02 and awaits their completion.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1420_H_
#define _P1420_H_

#include "class01.h"

class Class1420
{
public:
    Class1420(Class01 &object01, Class01& object02)
        : m_object01(object01)
        , m_object02(object02)
    {
    }

    async_task<int> coroutine1();
private:
    async_task<int> coroutine2();
    async_task<int> coroutine3();
    async_task<int> coroutine4();
    async_task<int> coroutine5();

public:
    Class01 m_object01;
    Class01 m_object02;
};

#endif