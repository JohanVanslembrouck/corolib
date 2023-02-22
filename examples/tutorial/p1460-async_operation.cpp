/**
 * @file p1460-async_operation.cpp
 * @brief
 *
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "class02.h"

Class02 object01;
Class02 object02;

// Uses coroutine1 implemented in p1460.cpp
async_task<int> coroutine1();

void completionflow()
{
    // Begin manual event completion
    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    for (int i = 0; i < 12; i++)
    {
        print(PRI1, "completionflow(): before object01.eventHandler[%d](10);\n", i);
        object01.eventHandler[i](10);
        print(PRI1, "completionflow(): after object01.eventHandler[%d](10);\n", i);

        print(PRI1, "completionflow(): before object02.eventHandler[%d](10);\n", i);
        object02.eventHandler[i](10);
        print(PRI1, "completionflow(): after object02.eventHandler[%d](10);\n", i);

        print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    // End manual event completion
}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    print(PRI1, "main(): async_task<int> a = coroutine1();\n");
    async_task<int> a = coroutine1();

    print(PRI1, "main(): completionflow();\n");
    completionflow();

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): return 0;\n");
    return 0;
}
