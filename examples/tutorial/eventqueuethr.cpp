/**
 * @file eventqueuethr.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */
 
 #include "eventqueuethr.h"
 
#include <corolib/print.h>

using namespace corolib;

void runEventQueue(EventQueueThrFunctionVoidInt& queue, int size, int val)
{
    for (int i = 0; i < size; i++)
    {
        print(PRI1, "runEventQueue(): std::function<void(int)> fun = queue.pop();\n");
        std::function<void(int)> op = queue.pop();
        print(PRI1, "runEventQueue(): op(%d);\n", val);
        op(val);
    }
}

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
