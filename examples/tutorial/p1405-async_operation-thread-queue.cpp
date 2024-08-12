/**
 * @file p1405-async_operation-thread-queue.cpp
 * @brief
 * Starts an asynchronous operation that will be completed after one second from a thread.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "p1400.h"
#include "eventqueuethr.h"

extern EventQueueThrFunctionVoidInt eventQueueThr;      // p1400.cpp

using namespace corolib;

void completionflow()
{
    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));

    runEventQueue(eventQueueThr, 2);
}

int main()
{
    useMode = UseMode::USE_THREAD_QUEUE;

    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        print(PRI1, "main(): async_task<int> a = coroutine1();\n");
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
