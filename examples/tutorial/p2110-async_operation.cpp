/**
 * @file p2110-async_operation.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <functional>
#include <future>

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "p2110.h"

void completionflow(CoroClass01& coroObject)
{
    // Begin manual event completion
    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow(): before object01.runEventHandler(10);\n");
    coroObject.m_object.runEventHandler(10);
    print(PRI1, "completionflow(): after object01.runEventHandler(10);\n");

    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    print(PRI1, "completionflow(): before object01.runEventHandler(10);\n");
    coroObject.m_object.runEventHandler(10);
    print(PRI1, "completionflow(): after object01.runEventHandler(10);\n");
    // End manual event completion
}

Class01 object01;
CoroClass01 coroObject01(object01);

Class01 object02;
CoroClass01 coroObject02(object02);

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        print(PRI1, "main(): auto task1thr = std::async(std::launch::async, task1, coroObject01);\n");
        auto task1thr = std::async(std::launch::async, task1, coroObject01);
        print(PRI1, "main(): auto task2thr = std::async(std::launch::async, task2, coroObject02);\n");
        auto task2thr = std::async(std::launch::async, task1, coroObject02);

        print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(0));

        print(PRI1, "main(): int v = task1thr.get() + task2thr.get();\n");
        int v = task1thr.get() + task2thr.get();

        print(PRI1, "main(): v = %d;\n", v);
    }

    print(PRI1, "main(): return 0;\n");
    return 0;
}
