/**
 * @file class01.h
 * @brief
 * Defines a class with (the simulation of) one asynchronous operation.
 *
 * @author Johan Vanslembrouck
 */
 
#ifndef _CLASS01_H_
#define _CLASS01_H_

#include <functional>

#include <corolib/commservice.h>
#include <corolib/async_operation.h>
#include <corolib/semaphore.h>
#include <corolib/threadawaker.h>

#include "eventqueue.h"
#include "eventqueuethr.h"
#include "use_mode.h"

using namespace corolib;

class Class01 : public CommService
{
public:
    Class01(
        UseMode useMode = UseMode::USE_NONE,
        EventQueueFunctionVoidInt* eventQueue = nullptr,
        EventQueueThrFunctionVoidInt* eventQueueThr = nullptr,
        std::mutex* mtx = nullptr,
        ThreadAwaker* awaker = nullptr,
        int delay = 0);

    async_operation<int> start_operation();
    
    void runEventHandler(int val)
    {
        // When the event handler runs, it may overwrite m_eventHandler.
        // This did not cause any problems so far. Anyway make a copy first.
        std::function<void(int)> eventHandler_ = m_eventHandler;
        eventHandler_(val);
    }

    EventQueueFunctionVoidInt* getEventQueue() { return m_eventQueue; }

protected:
    void async_op(std::function<void(int)>&& completionHandler);
	void start_operation_impl(async_operation<int>* om_async_operation_t);
    void start_operation_impl(const int idx);
    
private:
    std::function<void(int)> m_eventHandler;
    UseMode m_useMode;
    EventQueueFunctionVoidInt* m_eventQueue;
    EventQueueThrFunctionVoidInt* m_eventQueueThr;
    std::mutex* m_mutex;
    ThreadAwaker* m_awaker;
    int m_delay;
    int m_queueSize;
};

#endif
