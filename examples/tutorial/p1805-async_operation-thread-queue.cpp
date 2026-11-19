/**
 * @file p1805-async_operation-thread-queue.cpp
 * @brief
 * Starts an asynchronous operation that will be completed from the main() function.
 *
 * @author Johan Vanslembrouck
 */

#if !USE_1810
#include "p1800.h"
#define Class18XX Class1800
#else
#include "p1810.h"
#define Class18XX Class1810
#endif

#include "eventqueuethr.h"

using namespace corolib;

extern EventQueueThrFunctionVoidInt eventQueueThr;

void completionflow(Class18XX& obj)
{
    print(PRI1, "completionflow()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        print(PRI1, "completionflow(): obj.start_operation_impl();\n");
        obj.start_operation_impl();
    }

    print(PRI1, "completionflow(): runEventQueueThr(eventQueueThr, %d);\n", defaultCompletionValue);
    runEventQueueThr(eventQueueThr, defaultCompletionValue);

    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow(): before obj.op.set_result_and_complete(0);\n");
    obj.op.set_result_and_complete(0);
    print(PRI1, "completionflow(): after obj.op.set_result_and_complete(0);\n");
    // End manual event completion

    obj.op.reset();
}

int main()
{
    useMode = UseMode::USE_THREAD_QUEUE;

    set_priority(0x01);                     // Use 0x03 to follow the flow in corolib

    Class18XX obj;

    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
        async_task<int> a = obj.coroutine1();

        print(PRI1, "main(): completionflow(obj);\n");
        completionflow(obj);

        print(PRI1, "main(): int v = a.get_result();\n");
        int v = a.get_result();
        print(PRI1, "main(): v = %d\n", v);

        print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    print(PRI1, "main(): return 0;\n");
    return 0;
}
