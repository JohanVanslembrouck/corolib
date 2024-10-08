/**
 * @file p1439-async_operation-thread-imm.cpp
 * @brief
 *
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "class01.h"
#include "p1430.h"

void completionflow(ThreadAwaker* awaker)
{
    if (awaker)
        awaker->releaseThreads();
}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        ThreadAwaker awaker;
        Class01 object01(UseMode::USE_THREAD, nullptr, nullptr, nullptr, &awaker);
        Class01 object02(UseMode::USE_IMMEDIATE_COMPLETION);
        Class1430 obj{ object01, object02 };
        async_task<int> a = obj.coroutine1();

        print(PRI1, "main(): completionflow(1awaker);\n");
        completionflow(&awaker);

        print(PRI1, "main(): int v = a.get_result();\n");
        int v = a.get_result();
        print(PRI1, "main(): v = %d\n", v);

        print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(0));
    }

    print(PRI1, "main(): return 0;\n");
    return 0;
}
