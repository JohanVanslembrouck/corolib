/**
 * @file runeventqueue.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <corolib/print.h>

#include "runeventqueue.h"

namespace corolib
{
    // Use big size that supersedes the number of events in the applications
    void runEventQueue(CommQueue& queue, int size)
    {
        print(PRI5, "runEventQueue: begin\n");
        for (int i = 0; i < size; i++)
        {
            print(PRI5, "runEventQueue(): FunctionVoidVoid fun = queue.pop();\n");
            FunctionVoidVoid op = queue.pop();
            print(PRI5, "runEventQueue(): before op();\n");
            op();
            print(PRI5, "runEventQueue(): after op();\n");
            if (queue.stopped())
                break;
        }
        print(PRI5, "runEventQueue: end\n");
    }
}
