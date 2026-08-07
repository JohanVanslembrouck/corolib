/**
 * @file eventqueuethr.h
 * @brief
 * 
 * QueueThr is used in tutorial applications where completion takes place on a dedicated thread.
 * This dedicated completion thread places a functor onto the event queue.
 * The original launching thread pops the functors from the queue 
 * and calls the functors to resume the coroutines on the thread on which they were started.
 * 
 * For use in applications, please use QueueThreadSafe from include/corolib/eventqueue.h
 * 
 * @author Johan Vanslembrouck
 */
 
#ifndef _EVENTQUEUETHR_H_
#define _EVENTQUEUETHR_H_

#include "queuethr.h"

#include <functional>

constexpr int ARRAYSIZE = 16;   // Use 2^N

using EventQueueThrFunctionVoidInt = QueueThr<std::function<void(int)>, ARRAYSIZE>;
using EventQueueThrFunctionVoidVoid = QueueThr<std::function<void(void)>, ARRAYSIZE>;

void runEventQueue(EventQueueThrFunctionVoidInt& queue, int size, int val = 10);
void runEventQueue(EventQueueThrFunctionVoidVoid& queue, int size);

#endif
