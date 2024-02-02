/**
 * @file p1470.h
 * @brief
 * Example with 6 coroutines.
 * coroutineI (I = 1..4) co_awaits coroutineI+1.
 * In contrast to most other p14X0.cpp files, coroutine3 calls coroutine4 only once.
 * coroutine4 starts coroutine5a, coroutine5b and coroutine5c and awaits their completion.
 * coroutine5a and coroutine5b are very similar and run a potentially infinite loop using
 * a variable "running" that is initialized to true.
 * coroutine5c starts a thread that sleeps for 30 seconds.
 * When the thread returns from its sleep, coroutine5c continues and sets "running" to false.
 * coroutine5a, coroutine5b and coroutine5c co_return at this moment.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1470_H_
#define _P1470_H_

#include "class02.h"

class Class1470
{
public:
    Class1470(Class02& object01, Class02& object02)
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
    async_task<int> coroutine5c();

public:
    Class02 m_object01;
    Class02 m_object02;

    bool m_running = true;
};

#endif