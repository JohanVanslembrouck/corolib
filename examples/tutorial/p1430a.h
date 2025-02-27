/**
 * @file p1430a.h
 * @brief
 * Example with 6 coroutines.
 * coroutineI (I = 1..4) co_awaits coroutineI+1.
 * coroutine3 calls coroutine4 twice.
 * coroutine4 calls coroutine5a and coroutine5b and awaits the completion of both coroutines.
 * p1430a.cpp uses when_any instead of when_all (in p1430.cpp).
 * coroutine5a starts an asynchronous operation on object01 and awaits its completion.
 * coroutine5b starts an asynchronous operation on object02 and awaits its completion.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _P1430A_H_
#define _P1430A_H_

#include "class01.h"

class Class1430a
{
public:
    Class1430a(Class01& object01, Class01& object02)
        : m_object01(object01)
        , m_object02(object02)
    {
    }

    async_task<int> coroutine1();
    async_task<int> coroutine2();
    async_task<int> coroutine3();
    async_task<int> coroutine4();
    async_task<int> coroutine5a();
    async_task<int> coroutine5b();

public:
    Class01 &m_object01;
    Class01 &m_object02;
};

#endif