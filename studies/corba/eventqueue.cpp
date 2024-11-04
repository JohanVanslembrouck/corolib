/**
 * @file eventqueue.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include "eventqueue.h"

EventQueue eventqueue;

void EventQueue::run()
{
    while (!q.empty())
    {
        QueueElement qe = q.front();
        (*qe.handler)();
        q.pop();
    }
}

Handler_impl_ptr EventQueue::runOnce()
{
    if (!q.empty())
    {
        QueueElement qe = q.front();
        (*qe.handler)();
        q.pop();
        return qe.handler;
    }
    else
        return nullptr;
}

Handler_impl_ptr EventQueue::runOnce(PollerID pollerid)
{
    for (long unsigned int i = 0; i < q.size(); ++i)
    {
        QueueElement qe = q.front();
        (*qe.handler)();
        q.pop();

        if (qe.id == pollerid) {
            return qe.handler;
        }
        else {
            push(qe.handler, qe.id);
        }
    }
    return nullptr;
}

PollerID EventQueue::push(Handler_impl_ptr handler)
{
    pollerId++;
    q.push({ handler, pollerId });
    return pollerId;
}

void EventQueue::push(Handler_impl_ptr handler, PollerID pollerId)
{
    q.push({ handler, pollerId });
}
