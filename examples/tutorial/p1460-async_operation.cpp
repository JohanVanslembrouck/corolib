/**
 *  Filename: p1460-async_operation.cpp
 *  Description:
 *
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "class02.h"

Class02 object01;
Class02 object02;

async_task<int> coroutine1();

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    print(PRI1, "main(): async_task<int> a = coroutine1();\n");
    async_task<int> a = coroutine1();

    // Begin manual event completion
    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    for (int i = 0; i < 12; i++)
    {
        print(PRI1, "main(): before object01.operation[%d](10);\n", i);
        object01.operation[i](10);
        print(PRI1, "main(): after object01.operation[%d](10);\n", i);

        print(PRI1, "main(): before object02.operation[%d](10);\n", i);
        object02.operation[i](10);
        print(PRI1, "main(): after object02.operation[%d](10);\n", i);

        print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    // End manual event completion

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): return 0;\n");
    return 0;
}
