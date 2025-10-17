/**
 * @file eventqueue.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */
 
#include "corolib/print.h"
#include "corolib/eventqueue.h"

namespace corolib
{
    void runEventQueue(EventQueueFunctionVoidVoid& queue, int size)
    {
        clprint(PRI5, "runEventQueue: begin\n");
        for (int i = 0; i < size; i++)
        {
            clprint(PRI5, "runEventQueue(): std::function<void(void)> fun = queue.pop();\n");
            std::function<void(void)> op = queue.pop();
            clprint(PRI5, "runEventQueue(): before op();\n");
            op();
            clprint(PRI5, "runEventQueue(): after op();\n");
            if (queue.stopped())
                break;
        }
        // Process elements that are still in the queue
        int length = queue.length();
        clprint(PRI5, "runEventQueue: queue.length() = %zd\n", length);
        for (int i = 0; i < length; i++)
        {
            std::function<void(void)> op = queue.pop();
            clprint(PRI5, "runEventQueue(): before op();\n");
            op();
            clprint(PRI5, "runEventQueue(): after op();\n");
        }
        clprint(PRI5, "runEventQueue: end\n");
    }
}
