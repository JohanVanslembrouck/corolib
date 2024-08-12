/**
 * @file p2102-async_operation-eventqueue.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <functional>
#include <future>

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "p2100.h"

void completionflow()
{
    print(PRI1, "completionflow(): runEventQueue(*thr_object01.getEventQueue());\n");
    runEventQueue(*thr_object01.getEventQueue());
}

EventQueueFunctionVoidInt eventQueue1;
Class01 object01(UseMode::USE_EVENTQUEUE, &eventQueue1);

EventQueueFunctionVoidInt eventQueue2;
Class01 object02(UseMode::USE_EVENTQUEUE, &eventQueue2);

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        print(PRI1, "main(): auto task1thr = std::async(std::launch::async, task1, object01);\n");
        auto task1thr = std::async(std::launch::async, task1, object01);
        print(PRI1, "main(): auto task2thr = std::async(std::launch::async, task2, object02);\n");
        auto task2thr = std::async(std::launch::async, task1, object02);

        print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(0));

        print(PRI1, "main(): int v = task1thr.get() + task2thr.get();\n");
        int v = task1thr.get() + task2thr.get();

        print(PRI1, "main(): v = %d;\n", v);
    }

    print(PRI1, "main(): return 0;\n");
    return 0;
}
