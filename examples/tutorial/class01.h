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
#include "eventqueuethr.h"

using namespace corolib;

enum class UseMode
{
    USE_NONE,
    USE_EVENTQUEUE,
    USE_THREAD,
    USE_THREAD_QUEUE,
    USE_IMMEDIATE_COMPLETION
};

class Class01 : public CommService
{
public:
    Class01(UseMode useMode = UseMode::USE_NONE,
            EventQueueFunctionVoidInt* eventQueue = nullptr,
            EventQueueThrFunctionVoidInt* eventQueueThr = nullptr)
        : m_useMode(useMode)
        , m_eventQueue(eventQueue)
        , m_eventQueueThr(eventQueueThr)
        , m_queueSize(0)
    {
    }
    
    async_operation<int> start_operation();
    
    std::function<void(int)> eventHandler;
    EventQueueFunctionVoidInt* getEventQueue() { return m_eventQueue; }

protected:
    void async_op(std::function<void(int)>&& completionHandler);
    void start_operation_impl(const int idx);

private:
    UseMode     m_useMode;
    EventQueueFunctionVoidInt* m_eventQueue;
    EventQueueThrFunctionVoidInt* m_eventQueueThr;
    int m_queueSize;
};

#endif
