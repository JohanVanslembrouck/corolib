/**
 * @file runeventqueue.h
 * @brief This file defines QueueThr queues of different element types and 
 * functions to run over the elements in the queue.
 * All queues contain functor elements (std::function<TYPE>);
 * 
 * @author Johan Vanslembrouck
 */

#ifndef _RUNEVENTQUEUE_H_
#define _RUNEVENTQUEUE_H_

#include <functional>
#include <variant>

#include "eventqueuethr.h"

constexpr int ARRAYSIZE = 64;   // Use 2^N

using FunctionVoidVoid = std::function<void(void)>;
using EventQueueThrFunctionVoidVoid = QueueThr<FunctionVoidVoid, ARRAYSIZE>;
void runEventQueue(EventQueueThrFunctionVoidVoid& queue);

using FunctionVoidBool = std::function<void(bool)>;
using EventQueueThrFunctionVoidBool = QueueThr<FunctionVoidBool, ARRAYSIZE>;
void runEventQueue(EventQueueThrFunctionVoidBool& queue);

using FunctionVoidInt = std::function<void(int)>;
using EventQueueThrFunctionVoidInt = QueueThr<FunctionVoidInt, ARRAYSIZE>;
void runEventQueue(EventQueueThrFunctionVoidInt& queue);

using FunctionVoidString = std::function<void(std::string)>;
using EventQueueThrFunctionVoidString = QueueThr<FunctionVoidString, ARRAYSIZE>;
void runEventQueue(EventQueueThrFunctionVoidString& queue);


using FunctionVoidX = std::variant<FunctionVoidBool, FunctionVoidInt, FunctionVoidString, FunctionVoidVoid>;
using EventQueueThrFunctionX = QueueThr<FunctionVoidX, ARRAYSIZE>;
void runEventQueue(EventQueueThrFunctionX& queue);

#endif
