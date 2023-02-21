/**
 * @file eventqueuethr.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */
 
 #include "eventqueuethr.h"
 
#include <corolib/print.h>

using namespace corolib;

void runEventQueue(EventQueueThrFunctionVoidInt& queue, int size)
{
    for (int i = 0; i < size; i++)
    {
        print(PRI1, "runEventQueue(): std::function<void(int)> fun = queue.pop();\n");
        std::function<void(int)> op = queue.pop();
        print(PRI1, "runEventQueue(): op(10);\n");
        op(10);
    }
}
