/**
 * @file eventqueuethr.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include "corolib/print.h"
#include "eventqueuethr.h"

EventQueueThrFunctionVoidVoid evqueuethr;

using namespace corolib;

void runEventQueue(EventQueueThrFunctionVoidVoid& queue, int size)
{
    for (int i = 0; i < size; i++)
    {
        print(PRI1,  "runEventQueue(): std::function<void(void)> fun = queue.pop();\n");
        std::function<void(void)> op = queue.pop();
        print(PRI1, "runEventQueue(): op();\n");
        op();
    }
}
