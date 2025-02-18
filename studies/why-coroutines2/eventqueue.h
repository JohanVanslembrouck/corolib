/**
 * @file eventqueue.h
 * @brief
 * See the event queue description in tutorial/eventqueue.h
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#pragma once

#include <stdio.h>

#include <functional>
#include <queue>
#include <thread>

#include "corolib/print.h"

using namespace corolib;

using FunctionVoidVoid = std::function<void(void)>;
using FunctionVoidVoidPtr = std::function<void(void*)>;

template<typename TYPE>
class EventQueue
{
public:
    void run()
    {
        while (!q.empty())
        {
            print(PRI1, "-- eventqueue --\n");
            TYPE op = q.front();
            q.pop();
            op();
        }
    }

    void push(TYPE op)
    {
        q.push(op);
    }

private:
    std::queue<TYPE> q;
};

template<typename TYPE1, typename TYPE2>
class EventQueueX
{
private:
    struct queue_element
    {
        TYPE1   m_function;
        TYPE2   m_data;
    };
    std::queue<queue_element> q;

public:
    void run()
    {
        while (!q.empty())
        {
            print(PRI1, "-- eventqueueX --\n");
            queue_element op = q.front();
            q.pop();
            op.m_function(op.m_data);
        }
    }

    void push(queue_element op)
    {
        q.push(op);
    }
};

extern EventQueue<FunctionVoidVoid> evqueue;
extern EventQueueX<FunctionVoidVoidPtr, void*> evqueueX;
