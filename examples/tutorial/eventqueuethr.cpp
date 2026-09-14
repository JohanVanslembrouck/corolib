/**
 * @file eventqueuethr.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */
 
 #include "eventqueuethr.h"
 
#include <corolib/print.h>

using namespace corolib;

/**
 * Popping and executing a completion handler resumes code 
 * that may start other threads that push a completion handler.
 * That is why getPushCounter() must be called after each op(...) call.
 */

void runEventQueueThr(EventQueueThrFunctionVoidInt& queue, int val, int size)
{
    print(PRI2, "runEventQueueThr(): size = %d\n", queue.getPushCounter());
    if (size == -1)
    {
        size = queue.getPushCounter();
        while (size > 0)
        { 
            print(PRI1, "runEventQueueThr(): std::function<void(int)> fun = queue.pop();\n");
            std::function<void(int)> op = queue.pop();
            print(PRI1, "runEventQueueThr(): op(%d);\n", val);
            op(val);
            size = queue.getPushCounter();
        }
    }
    else
    {
        for (int i = 0; i < size; i++)
        {
            print(PRI1, "runEventQueueThr(): std::function<void(int)> fun = queue.pop();\n");
            std::function<void(int)> op = queue.pop();
            print(PRI1, "runEventQueueThr(): op(%d);\n", val);
            op(val);
        }
    }
}

void runEventQueueThr(EventQueueThrFunctionVoidVoid& queue, int size)
{
    if (size == -1)
    {
        size = queue.getPushCounter();
        while (size > 0)
        {
            print(PRI1, "runEventQueue(): std::function<void(int)> fun = queue.pop();\n");
            std::function<void(void)> op = queue.pop();
            print(PRI1, "runEventQueue(): op();\n");
            op();
            size = queue.getPushCounter();
        }
    }
    else
    {
        for (int i = 0; i < size; i++)
        {
            print(PRI1, "runEventQueue(): std::function<void(void)> fun = queue.pop();\n");
            std::function<void(void)> op = queue.pop();
            print(PRI1, "runEventQueue(): op();\n");
            op();
        }
    }
}
