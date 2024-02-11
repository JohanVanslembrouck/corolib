/**
 * @file p1454-async_operation-thread.cpp
 * @brief
 *
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "p1450.h"

void completionflow()
{

}

int main()
{
    set_print_level(0x00);        // Use 0x03 to follow the flow in corolib

    Semaphore sema{ 1 };
    Class01 object01(UseMode::USE_THREAD, nullptr, nullptr, &sema);
    Class01 object02(UseMode::USE_THREAD, nullptr, nullptr, &sema);
    Class1450 obj{ object01, object02 };
    async_task<int> a = obj.coroutine1();

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
