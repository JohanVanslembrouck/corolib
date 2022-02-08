/**
 *  Filename: class02.h
 *  Description:
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 *
 */
 
#ifndef _CLASS02_H_
#define _CLASS02_H_

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
    USE_THREAD
};

class Class02 : public CommService
{
public:
    Class02(UseMode useMode = USE_NONE) 
        : m_useMode(useMode)
    {
    }
    
    async_operation<int> start_operation1();
    async_operation<int> start_operation2(int bias = 0);
    
    std::function<void(int)> operation[NROPERATIONS];

protected:
    void start_op1(const int idx);
    void start_op2(const int idx, int bias);

private:
    UseMode    m_useMode;
};

#endif
