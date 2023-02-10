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
    Class01(UseMode useMode = USE_NONE, EventQueue* eventQueue = nullptr)
        : m_useMode(useMode)
        , m_eventQueue(eventQueue)
    {
    }
    
    async_operation<int> start_operation();
    
    std::function<void(int)> eventHandler;
    EventQueue* getEventQueue() { return m_eventQueue; }

protected:
    void async_op(std::function<void(int)>&& completionHandler);
    void start_operation_impl(const int idx);

private:
    UseMode     m_useMode;
    EventQueue* m_eventQueue;
};

#endif
