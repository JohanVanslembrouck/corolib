/**
 * @file p1474-async_operation-thread.cpp
 * @brief
 *
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "p1470.h"

void completionflow(ThreadAwaker& awaker)
{
    awaker.releaseThreads();
}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    std::mutex mtx;
    ThreadAwaker awaker;
    Class02 object01(UseMode::USE_THREAD, nullptr, nullptr, &mtx, &awaker);
    Class02 object02(UseMode::USE_THREAD, nullptr, nullptr, &mtx, &awaker);
    Class1470 obj{ object01, object02 };
    async_task<int> a = obj.coroutine1();

    print(PRI1, "main(): completionflow(awaker);\n");
    completionflow(awaker);

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): return 0;\n");
    return 0;
}
