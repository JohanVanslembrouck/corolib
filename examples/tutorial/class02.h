/**
 * @file class02.h
 * @brief
 * Defines a class with (the simulation of) two asynchronous operations.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */
 
#ifndef _CLASS02_H_
#define _CLASS02_H_

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

class Class02 : public CommService
{
public:
    Class02(UseMode useMode = UseMode::USE_NONE,
        EventQueueFunctionVoidInt* eventQueue = nullptr,
        EventQueueThrFunctionVoidInt* eventQueueThr = nullptr)
        : m_useMode(useMode)
        , m_eventQueue(eventQueue)
        , m_eventQueueThr(eventQueueThr)
        , m_queueSize(0)
    {
    }
    
    async_operation<int> start_operation1();
    async_operation<int> start_operation2(int bias = 0);
    
    std::function<void(int)> eventHandler[NROPERATIONS];
    EventQueueFunctionVoidInt* getEventQueue() { return m_eventQueue; }

protected:
    void async_op1(const int idx, std::function<void(int)>&& completionHandler);
    void start_operation1_impl(const int idx);

    void async_op2(const int idx, int bias, std::function<void(int)>&& completionHandler);
    void start_operation2_impl(const int idx, int bias);

private:
    UseMode     m_useMode;
    EventQueueFunctionVoidInt* m_eventQueue;
    EventQueueThrFunctionVoidInt* m_eventQueueThr;
    int m_queueSize;
};

#endif
