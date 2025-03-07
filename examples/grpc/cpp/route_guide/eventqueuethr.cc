/**
 * @file eventqueuethr.cc
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */
 
#include "eventqueuethr.h"

#include <corolib/print.h>

using namespace corolib;

void runEventQueue(EventQueueThrFunctionVoidVoid& queue, int size)
{
    print(PRI1, "runEventQueue: begin\n");
    for (int i = 0; i < size; i++)
    {
        print(PRI5, "runEventQueue(): std::function<void(void)> fun = queue.pop();\n");
        std::function<void(void)> op = queue.pop();
        print(PRI5, "runEventQueue(): before op();\n");
        op();
        print(PRI5, "runEventQueue(): after op();\n");
        if (queue.stopped())
            break;
    }
    print(PRI1, "runEventQueue: end\n");
}
