/**
 * @file p1802-async_operation-eventqueue.cpp
 * @brief
 * Starts an asynchronous operation that will be completed from the main() function.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <functional>
#include <thread>

#include "p1800.h"
#include "eventqueue.h"

using namespace corolib;

extern EventQueueFunctionVoidInt eventQueue;                       // p1800.cpp

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
    print(PRI1, "completionflow(): before op.set_result_and_complete(0);\n");
    op.set_result_and_complete(0);
    print(PRI1, "completionflow(): after op.set_result_and_complete(0);\n");
    // End manual event completion
}

int main()
{
    useMode = UseMode::USE_EVENTQUEUE;

    set_priority(0x01);                     // Use 0x03 to follow the flow in corolib

    for (int i = 0; i < 1; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        print(PRI1, "main(): async_ltask<int> a = coroutine1();\n");
        async_task<int> a = coroutine1();

        print(PRI1, "main(): completionflow();\n");
        completionflow();

        print(PRI1, "main(): int v = a.get_result();\n");
        int v = a.get_result();
        print(PRI1, "main(): v = %d\n", v);

        print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(0));
    }

    print(PRI1, "main(): return 0;\n");
    return 0;
}
