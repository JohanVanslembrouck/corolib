/**
 * @file class01.h
 * @brief
 * Defines a class with (the simulation of) one asynchronous operation.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */
 
#ifndef _CLASS01_H_
#define _CLASS01_H_

#include <functional>

#include <corolib/commservice.h>
#include <corolib/async_operation.h>

#include "eventqueue.h"

extern EventQueue eventQueue;

using namespace corolib;

enum UseMode
{
    USE_NONE,
    USE_EVENTQUEUE,
    USE_THREAD,
    USE_IMMEDIATE_COMPLETION
};

class Class01 : public CommService
{
public:
    Class01(UseMode useMode = USE_NONE) 
        : m_useMode(useMode)
    {
    }
    
    async_operation<int> start_operation();
    
    std::function<void(int)> eventHandler;

protected:
    void start_operation_impl(const int idx);

private:
    UseMode    m_useMode;
};

#endif
