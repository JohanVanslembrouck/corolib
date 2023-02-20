/**
 * @file p1822-async_operation-eventqueue.cpp
 * @brief
 * Starts an asynchronous operation that will be completed from the main() function.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <functional>

#include "p1820.h"
#include "eventqueue.h"

using namespace corolib;

UseMode useMode = USE_EVENTQUEUE;

extern EventQueueFunctionVoidInt eventQueue;                       // p1820.cpp

void completionflow()
{
    print(PRI1, "completionflow()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow(): start_operation_impl(op);\n");
        start_operation_impl(op);
    }

    print(PRI1, "completionflow(): runEventQueue(eventQueue);\n");
    runEventQueue(eventQueue);

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow(): before op.set_result_and_complete(std::nullopt);\n");
    op.set_result_and_complete(std::nullopt);
    print(PRI1, "completionflow(): after op.set_result_and_complete(std::nullopt);\n");
    // End manual event completion
}

int main()
{
    set_priority(0x01);        // Use 0x03 to follow the flow in corolib

    print(PRI1, "main(): async_ltask<int> a = coroutine1();\n");
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
