/**
 * @file eventqueue.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _EVENTQUEUE_H_
#define _EVENTQUEUE_H_

#include <stdio.h>
#include <queue>

#include "handler.h"

using namespace CORBA;

class EventQueue
{
public:
    struct QueueElement
    {
        Handler_impl_ptr handler;
        PollerID id;
    };

    void run();
    Handler_impl_ptr runOnce();
    Handler_impl_ptr runOnce(PollerID pollerid);

    PollerID push(Handler_impl_ptr handler);
    void push(Handler_impl_ptr handler, PollerID pollerId);
    
    PollerID registerCB(Handler_impl_ptr handler)
    {
        return push(handler);
    }

private:
    PollerID pollerId = 0;
    std::queue<QueueElement> q;
};

extern EventQueue eventqueue;

#endif
