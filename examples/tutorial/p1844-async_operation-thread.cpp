/**
 * @file p1844-async_operation-thread.cpp
 * @brief
 * 
 *
 * @author Johan Vanslembrouck
 */

#include <thread>

#include <corolib/when_all.h>

using namespace corolib;

#include "p1840.h"
#include "class00.h"

async_task<void> completionflow(Class1840& obj, async_task<int>& a1, async_task<int>& a2, async_task<int>& a3)
{
    print(PRI1, "completionflow()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        print(PRI1, "completionflow(): obj.start_operation_impl();\n");
        obj.start_operation_impl();
    }

    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow(): before obj.op1.set_result_and_complete(std::nullopt);\n");
    obj.op1.set_result_and_complete(std::nullopt);
    print(PRI1, "completionflow(): after obj.op1.set_result_and_complete(std::nullopt);\n");
    // End manual event completion

    print(PRI1, "completionflow(): when_all wa(a1, a2, a3);\n");
    when_all wa(a1, a2, a3);
    print(PRI1, "completionflow(): co_await wa;\n");
    co_await wa;
}

int main()
{
    useMode = UseMode::USE_THREAD;

    set_priority(0x01);        // Use 0x03 to follow the flow in corolib

    delay = 0;

    Class1840 obj;

    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        print(PRI1, "main(): async_task<int> a1 = obj.coroutine1();\n");
        async_task<int> a1 = obj.coroutine1();
        print(PRI1, "main(): async_task<int> a2 = obj.coroutine2();\n");
        async_task<int> a2 = obj.coroutine2();
        print(PRI1, "main(): async_task<int> a3 = obj.coroutine3();\n");
        async_task<int> a3 = obj.coroutine3();

        print(PRI1, "main(): completionflow(obj, a1, a2, a3);\n");
        completionflow(obj, a1, a2, a3);

        print(PRI1, "main(): int v = a1.get_result() + a2.get_result() + a3.get_result();\n");
        int v = a1.get_result() + a2.get_result() + a3.get_result();
        print(PRI1, "main(): v = %d\n", v);

        print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(0));
    }

    print(PRI1, "main(): return 0;\n");
    return 0;
}
