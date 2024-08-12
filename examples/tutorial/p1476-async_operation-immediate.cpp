/**
 * @file p1476-async_operation-immediate.cpp
 * @brief
 *
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "p1470.h"

void completionflow()
{

}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    for (int i = 0; i < 3; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        Class02 object01(UseMode::USE_IMMEDIATE_COMPLETION);
        Class02 object02(UseMode::USE_IMMEDIATE_COMPLETION);
        Class1470 obj{ object01, object02 };
        async_task<int> a = obj.coroutine1();

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
