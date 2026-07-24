/**
 * @file runeventqueue.h
 * @brief Obsolete file
 *
 * @author Johan Vanslembrouck
 */

#ifndef _RUNEVENTQUEUE_H_
#define _RUNEVENTQUEUE_H_

#include <corolib/print.h>

constexpr int ARRAYSIZE = 16;   // Use 2^N

using EventQueueFunctionVoidVoid = QueueThreadSafe<std::function<void(void)>, ARRAYSIZE>;

// Use big size that supersedes the number of events in the applications
void runEventQueue(EventQueueFunctionVoidVoid& queue, int size = 100000)
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

#endif
