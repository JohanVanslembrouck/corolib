/**
 * @file eventqueuethr.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */
 
#include "eventqueuethr.h"

#include "print.h"

void runEventQueue(EventQueueThrFunctionVoidVoid& queue, int size)
{
    for (int i = 0; i < size; i++)
    {
        print(PRI1, "runEventQueue(): std::function<void(void)> fun = queue.pop();\n");
        std::function<void(void)> op = queue.pop();
        print(PRI1, "runEventQueue(): op();\n");
        op();
    }
}
