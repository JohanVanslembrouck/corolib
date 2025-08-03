/**
 * @file eventqueue.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */
 
#include <corolib/eventqueue.h>
#include <corolib/print.h>

namespace corolib
{
    void runEventQueue(EventQueueFunctionVoidVoid& queue, int size)
    {
        print(PRI5, "runEventQueue: begin\n");
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
        // Process elements that are still in the queue
        int length = queue.length();
        print(PRI5, "runEventQueue: queue.length() = %zd\n", length);
        for (int i = 0; i < length; i++)
        {
            std::function<void(void)> op = queue.pop();
            print(PRI5, "runEventQueue(): before op();\n");
            op();
            print(PRI5, "runEventQueue(): after op();\n");
        }
        print(PRI5, "runEventQueue: end\n");
    }
}
