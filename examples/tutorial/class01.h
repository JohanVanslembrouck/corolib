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
#include "use_mode.h"

using namespace corolib;

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
    void setThreadDelay(int delay) { m_delay = delay; }

    async_operation<int> start_operation();
    
    void runEventHandler(int val)
    {
        std::function<void(int)> eventHandler_ = eventHandler;
        eventHandler_(val);
    }

    EventQueueFunctionVoidInt* getEventQueue() { return m_eventQueue; }

protected:
    void async_op(std::function<void(int)>&& completionHandler);
    void start_operation_impl(const int idx);

public: // to be changed to private
    std::function<void(int)> eventHandler;
private:
    UseMode     m_useMode;
    EventQueueFunctionVoidInt* m_eventQueue;
    EventQueueThrFunctionVoidInt* m_eventQueueThr;
    int m_queueSize;
    int m_delay = 10;
};

#endif
