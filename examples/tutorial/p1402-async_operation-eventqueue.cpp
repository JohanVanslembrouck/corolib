/**
 * @file p1402-async_operation-eventqueue.cpp
 * @brief
 * Starts an asynchronous operation that will be completed from the main() function running the event queue.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "p1400.h"
#include "eventqueue.h"

using namespace corolib;

UseMode useMode = USE_EVENTQUEUE;

extern EventQueueFunctionVoidInt eventQueue;           // p1400.cpp

void completionflow()
{
    print(PRI1, "completionflow(): runEventQueue(eventQueue);\n");
    runEventQueue(eventQueue);
}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    print(PRI1, "main(): async_task<int> a = coroutine1();\n");
    async_task<int> a = coroutine1();

    print(PRI1, "main(): completionflow();\n");
    completionflow();

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): return 0;\n");
    return 0;
}
