/**
 * @file p1844-async_operation-thread.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <functional>

#include "p1840.h"
#include "eventqueue.h"

#include <corolib/when_all.h>

using namespace corolib;

async_task<void> completionflow(async_task<int>& a1, async_task<int>& a2, async_task<int>& a3)
{
    print(PRI1, "completionflow()\n");

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        print(PRI1, "completionflow(): start_operation_impl(op);\n");
        start_operation_impl(op1);
    }

    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Begin manual event completion to make coroutine1 co_return
    print(PRI1, "completionflow(): before op.set_result_and_complete(std::nullopt);\n");
    op1.set_result_and_complete(std::nullopt);
    print(PRI1, "completionflow(): after op.set_result_and_complete(std::nullopt);\n");
    // End manual event completion

    print(PRI1, "completionflow(): when_all wa({ &a1, &a2, &a3 });\n");
    when_all wa({ &a1, &a2, &a3 });
    print(PRI1, "completionflow(): co_await wa;\n");
    co_await wa;
}

int main()
{
    useMode = UseMode::USE_THREAD;

    set_priority(0x01);        // Use 0x03 to follow the flow in corolib

    print(PRI1, "main(): async_ltask<int> a = coroutine1();\n");
    async_task<int> a1 = coroutine1();
    async_task<int> a2 = coroutine2();
    async_task<int> a3 = coroutine3();
 
    print(PRI1, "main(): completionflow(a1, a2, a3);\n");
    completionflow(a1, a2, a3);
 
    print(PRI1, "main(): int v = a1.get_result() + a2.get_result() + a3.get_result();\n");
    int v = a1.get_result() + a2.get_result() + a3.get_result();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): return 0;\n");
    return 0;
}
