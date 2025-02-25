/**
 * @file p1480.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _P1480_H_
#define _P1480_H_

#include <random>
#include <string>
#include <thread>
#include <vector>

#include <corolib/print.h>
#include <corolib/commservice.h>
#include <corolib/async_task.h>
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

class Sorter : public CommService
{
public:
    Sorter(UseMode useMode = UseMode::USE_NONE,
        EventQueueFunctionVoidVoid* eventQueue = nullptr,
        EventQueueThrFunctionVoidVoid* eventQueueThr = nullptr)
        : m_useMode(useMode)
        , m_eventQueue(eventQueue)
        , m_eventQueueThr(eventQueueThr)
        , m_queueSize(0)
    {
    }

    virtual ~Sorter() {}

    async_operation<void> start_sorting(auto begin, auto end);

protected:
    void start_sort(int idx, auto begin, auto end);

private:
    UseMode     m_useMode;
    EventQueueFunctionVoidVoid* m_eventQueue;
    EventQueueThrFunctionVoidVoid* m_eventQueueThr;
    int m_queueSize;
};

async_task<void> sortCoroutine(Sorter& sorter, std::vector<int>& values);

#endif
