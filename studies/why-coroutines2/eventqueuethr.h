/**
 * @file eventqueuethr.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */
 
#pragma once

#include "queuethr.h"

#include <functional>

using FunctionVoidVoid = std::function<void(void)>;
using FunctionVoidVoidPtr = std::function<void(void*)>;

constexpr int ARRAYSIZE = 16;   // Use 2^N

using EventQueueThrFunctionVoidVoid = QueueThr<std::function<void(void)>, ARRAYSIZE>;

extern EventQueueThrFunctionVoidVoid evqueuethr;

void runEventQueue(EventQueueThrFunctionVoidVoid& queue, int size);
