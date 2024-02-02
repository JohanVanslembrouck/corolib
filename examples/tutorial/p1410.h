/**
 * @file p1410.h
 * @brief
 * Example with 5 coroutines.
 * coroutineI (I = 1..4) co_awaits coroutineI+1.
 * coroutine3 calls coroutine4 twice.
 * coroutine5 starts an asynchronous operation on object01 and awaits its completion.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1410_H_
#define _P1410_H_

#include "class01.h"

class Class1410
{
public:
    Class1410(Class01 &object)
        : m_object01(object)
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
};

#endif