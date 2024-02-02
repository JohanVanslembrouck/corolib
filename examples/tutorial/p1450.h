/**
 * @file p1450.h
 * @brief
 * Example with 6 coroutines.
 * coroutineI (I = 1..4) co_awaits coroutineI+1.
 * coroutine3 calls coroutine4 twice.
 * coroutine4 calls coroutine5a and coroutine5b and awaits the completion of both coroutines.
 * coroutine5a starts twice an asynchronous operation on object01 and awaits its completion.
 * coroutine5b starts twice an asynchronous operation on object02 and awaits its completion.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1450_H_
#define _P1450_H_

#include "class01.h"

class Class1450
{
public:
    Class1450(Class01& object01, Class01& object02)
        : m_object01(object01)
        , m_object02(object02)
    {
    }

    async_task<int> coroutine1();
private:
    async_task<int> coroutine2();
    async_task<int> coroutine3();
    async_task<int> coroutine4();
    async_task<int> coroutine5a();
    async_task<int> coroutine5b();

public:
    Class01 m_object01;
    Class01 m_object02;
};

#endif