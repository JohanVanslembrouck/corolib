/**
 * @file p1412-async_operation-eventqueue.cpp
 * @brief
 *
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "p1410.h"

EventQueueFunctionVoidInt eventQueue;

void completionflow()
{
    print(PRI1, "completionflow(): runEventQueue(eventQueue);\n");
    runEventQueue(eventQueue);
}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        Class01 object01(UseMode::USE_EVENTQUEUE, &eventQueue);
        Class1410 obj{ object01 };
        async_task<int> a = obj.coroutine1();

        print(PRI1, "main(): completionflow();\n");
        completionflow();

        print(PRI1, "main(): int v = awa.get_result();\n");
        int v = a.get_result();
        print(PRI1, "main(): v = %d\n", v);

        print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(0));
    }

    print(PRI1, "main(): return 0;\n");
    return 0;
}
