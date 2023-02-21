/**
 * @file eventqueue.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
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
void runEventQueue(EventQueueFunctionVoidInt& queue)
{
    while (!queue.empty())
    {
        print(PRI1, "runEventQueue(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        print(PRI1, "runEventQueue(): std::function<void(int)> op = queue.pull();\n");
        std::function<void(int)> op = queue.pull();
        print(PRI1, "runEventQueue(): op(10);\n");
        op(10);
    }
}
