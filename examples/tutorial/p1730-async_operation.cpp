/**
 * @file p1730-async_operation.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "class01.h"

Class01 object01;
Class01 object02;

// Uses coroutine1 implemented in p1730.cpp
async_ltask<int> coroutine1();

void completionflow()
{
    // Begin manual event completion
    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));

    print(PRI1, "completionflow(): before object01.runEventHandler(10);\n");
    object01.runEventHandler(10);
    print(PRI1, "completionflow(): after object01.runEventHandler(10);\n");

    print(PRI1, "completionflow(): before object02.runEventHandler(10);\n");
    object02.runEventHandler(10);
    print(PRI1, "completionflow(): after object02.runEventHandler(10);\n");

    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));

    print(PRI1, "completionflow(): before object02.runEventHandler(10);\n");
    object02.runEventHandler(10);
    print(PRI2, "completionflow(): after object02.runEventHandler(10);\n");

    print(PRI1, "completionflow(): before object01.runEventHandler(10);\n");
    object01.runEventHandler(10);
    print(PRI1, "completionflow(): after object01.runEventHandler(10);\n");

    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));
    // End manual event completion
}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        print(PRI1, "main(): async_ltask<int> a = coroutine1();\n");
        async_ltask<int> a = coroutine1();
        print(PRI1, "main(): a.start();\n");
        a.start();

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
