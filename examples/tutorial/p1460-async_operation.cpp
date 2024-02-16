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

#include "p1460.h"

void completionflow(Class1460& obj)
{
    // Begin manual event completion
    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    for (int i = 0; i < 12; i++)
    {
        print(PRI1, "completionflow(): before obj.m_object01.runEventHandler(%d, 10);\n", i);
        obj.m_object01.runEventHandler(i, 10);
        print(PRI1, "completionflow(): after obj.m_object01.runEventHandler(%d, 10);\n", i);

        print(PRI1, "completionflow(): before obj.m_object02.runEventHandler(%d, 10);\n", i);
        obj.m_object02.runEventHandler(i, 10);
        print(PRI1, "completionflow(): after obj.m_object02.runEventHandler(%d, 10);\n", i);

        print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    // End manual event completion
}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    Class02 object01;
    Class02 object02;
    Class1460 obj{ object01, object02 };
    async_task<int> a = obj.coroutine1();

    print(PRI1, "main(): completionflow(obj);\n");
    completionflow(obj);

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): return 0;\n");
    return 0;
}
