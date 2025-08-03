/**
 * @file p2012-async_operation-eventqueue.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <functional>
#include <future>

#include "p2010.h"
#include "eventqueue.h"

using namespace corolib;

extern EventQueueFunctionVoidInt eventQueue;                       // p2010.cpp

void completionflow()
{
    print(PRI1, "completionflow()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow(): start_operation_impl(op1);\n");
        start_operation_impl(op1);
    }

    print(PRI1, "completionflow(): runEventQueue(eventQueue), %d;\n", defaultCompletionValue);
    runEventQueue(eventQueue, defaultCompletionValue);

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow(): before op.set_result_and_complete(std::nullopt);\n");
    op1.set_result_and_complete(std::nullopt);
    print(PRI1, "completionflow(): after op.set_result_and_complete(std::nullopt);\n");
    // End manual event completion
}

int main()
{
    useMode = UseMode::USE_EVENTQUEUE;

    set_priority(0x01);        // Use 0x03 to follow the flow in corolib

    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        print(PRI1, "main(): auto task1thr = std::async(std::launch::async, task1);\n");
        auto task1thr = std::async(std::launch::async, task1);
        print(PRI1, "main(): auto task2thr = std::async(std::launch::async, task2);\n");
        auto task2thr = std::async(std::launch::async, task2);
        print(PRI1, "main(): auto task3thr = std::async(std::launch::async, task3);\n");
        auto task3thr = std::async(std::launch::async, task3);

        print(PRI1, "main(): completionflow();\n");
        completionflow();

        print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(0));

        print(PRI1, "main(): int v = task1thr.get() + task2thr.get() + task3thr.get();\n");
        int v = task1thr.get() + task2thr.get() + task3thr.get();

        print(PRI1, "main(): v = %d\n", v);
    }

    print(PRI1, "main(): return 0;\n");
    return 0;
}
