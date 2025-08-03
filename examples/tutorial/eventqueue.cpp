/**
 * @file eventqueue.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <thread>

#include "eventqueue.h"

#include <corolib/print.h>

using namespace corolib;

/**
  * @brief runEventQueue takes a functor from the front 
  * of the internal queue of void(int) functors
  * and calls this functor with argument = 10.
  * run uses a delay of 1000 ms before each functor invocation.
  */
void runEventQueue(EventQueueFunctionVoidInt& queue, int val)
{
    while (!queue.empty())
    {
        print(PRI1, "runEventQueue(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        print(PRI1, "runEventQueue(): std::function<void(int)> op = queue.pop();\n");
        std::function<void(int)> op = queue.pop();
        print(PRI1, "runEventQueue(): op(%d);\n", val);
        op(val);
    }
}

void runEventQueue(EventQueueFunctionVoidVoid& queue)
{
    while (!queue.empty())
    {
        print(PRI1, "runEventQueue(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        print(PRI1, "runEventQueue(): std::function<void(int)> op = queue.pop();\n");
        std::function<void(void)> op = queue.pop();
        print(PRI1, "runEventQueue(): op();\n");
        op();
    }
}

