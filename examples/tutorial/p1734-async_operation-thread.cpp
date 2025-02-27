/**
 * @file p1734-async_operation-thread.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "p1730.h"

void completionflow()
{
}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        Class01 object01(UseMode::USE_THREAD, nullptr, nullptr, nullptr, nullptr);
        Class01 object02(UseMode::USE_THREAD, nullptr, nullptr, nullptr, nullptr);
        Class1730 obj{ object01, object02 };
        print(PRI1, "main(): async_ltask<int> a = obj.coroutine1();\n");
        async_ltask<int> a = obj.coroutine1();
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
