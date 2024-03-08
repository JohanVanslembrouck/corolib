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
#include <corolib/semaphore.h>
#include <corolib/threadawaker.h>

#include "eventqueue.h"
#include "eventqueuethr.h"
#include "use_mode.h"

using namespace corolib;

class Class02 : public CommService
{
public:
    Class02(UseMode useMode = UseMode::USE_NONE,
            EventQueueFunctionVoidInt* eventQueue = nullptr,
            EventQueueThrFunctionVoidInt* eventQueueThr = nullptr,
            std::mutex* mtx = nullptr,
            ThreadAwaker* awaker = nullptr,
            int delay = 10)
        : m_useMode(useMode)
        , m_eventQueue(eventQueue)
        , m_eventQueueThr(eventQueueThr)
        , m_mutex(mtx)
        , m_awaker(awaker)
        , m_delay(delay)
        , m_queueSize(0)
    {
    }

    async_operation<int> start_operation1();
    async_operation<int> start_operation2(int bias = 0);
    
    void runEventHandler(int i, int val)
    {
        // When the event handler runs, it may overwrite m_eventHandler[i].
        // This did not cause any problems so far. Anyway make a copy first.
        std::function<void(int)> eventHandler_ = m_eventHandler[i];
        eventHandler_(val);
    }

    EventQueueFunctionVoidInt* getEventQueue() { return m_eventQueue; }

protected:
    void async_op1(const int idx, std::function<void(int)>&& completionHandler);
    void start_operation1_impl(const int idx);

    void async_op2(const int idx, int bias, std::function<void(int)>&& completionHandler);
    void start_operation2_impl(const int idx, int bias);
    
private:
    std::function<void(int)> m_eventHandler[NROPERATIONS];
    UseMode m_useMode;
    EventQueueFunctionVoidInt* m_eventQueue;
    EventQueueThrFunctionVoidInt* m_eventQueueThr;
    std::mutex* m_mutex;
    ThreadAwaker* m_awaker;
    int m_delay;
    int m_queueSize;
};

#endif
