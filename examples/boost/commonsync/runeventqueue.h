/**
 * @file runeventqueue.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#ifndef _RUNEVENTQUEUE_H_
#define _RUNEVENTQUEUE_H_

#include "functional"

#include <corolib/eventqueue.h>

namespace corolib
{
    constexpr int ARRAYSIZE = 16;   // Use 2^N

    using FunctionVoidVoid = std::function<void(void)>;
    using EventQueueFunctionVoidVoid = QueueThreadSafe<FunctionVoidVoid, ARRAYSIZE>;
    using CommQueue = EventQueueFunctionVoidVoid;

    void runEventQueue(CommQueue& queue, int size = 100000);
}

#endif
