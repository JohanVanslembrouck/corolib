/**
 *  Filename: class01.h
 *  Description:
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 *
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
    
    std::function<void(int)> operation;

protected:
    void start_op(const int idx);

private:
    UseMode    m_useMode;
};

#endif
