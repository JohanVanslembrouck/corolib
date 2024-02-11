/**
 * @file p2005-async_operation-thread-queue.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <functional>
#include <thread>

#include "p2000.h"
#include "eventqueuethr.h"

using namespace corolib;

extern EventQueueThrFunctionVoidInt eventQueueThr;                       // p2000.cpp

void task0()
{
    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        print(PRI1, "completionflow(): start_operation_impl(op);\n");
        start_operation_impl(op1);
    }
}

void completionflow()
{
    print(PRI1, "completionflow()\n");

    print(PRI1, "completionflow(): std::jthread task1thr{ task0 };\n");
    std::jthread task1thr{ task0 };

    print(PRI1, "completionflow(): runEventQueue(eventQueueThr, 4);\n");
    runEventQueue(eventQueueThr, 4);

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow(): before op.set_result_and_complete(std::nullopt);\n");
    op1.set_result_and_complete(std::nullopt);
    print(PRI1, "completionflow(): after op.set_result_and_complete(std::nullopt);\n");
    // End manual event completion
}

int main()
{
    useMode = UseMode::USE_THREAD_QUEUE;

    set_priority(0x01);        // Use 0x03 to follow the flow in corolib

    print(PRI1, "main(): std::jthread task1thr{ task1 };\n");
    std::jthread task1thr{ task1 };
    print(PRI1, "main(): std::jthread task2thr{ task2 };\n");
    std::jthread task2thr{ task2 };
    print(PRI1, "main(): std::jthread task3thr{ task3 };\n");
    std::jthread task3thr{ task3 };

    print(PRI1, "main(): completionflow();\n");
    completionflow();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): return 0;\n");
    return 0;
}
