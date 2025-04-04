/**
 * @file p1730a-async_operation.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "p1730a.h"

void completionflow(Class1730a& obj)
{
    // Begin manual event completion
    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));

    print(PRI1, "completionflow(): 1. before obj.m_object01.runEventHandler(10);\n");
    obj.m_object01.runEventHandler(10);
    print(PRI1, "completionflow(): 1. after obj.m_object01.runEventHandler(10);\n");

    print(PRI1, "completionflow(): 2. before obj.m_object02.runEventHandler(10);\n");
    obj.m_object02.runEventHandler(10);
    print(PRI1, "completionflow(): 2. after obj.m_object02.runEventHandler(10);\n");

    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));

    print(PRI1, "completionflow(): 3. before obj.m_object02.runEventHandler(10);\n");
    obj.m_object02.runEventHandler(10);
    print(PRI2, "completionflow(): 3. after obj.m_object02.runEventHandler(10);\n");

    print(PRI1, "completionflow(): 4. before obj.m_object01.runEventHandler(10);\n");
    obj.m_object01.runEventHandler(10);
    print(PRI1, "completionflow(): 4. after obj.m_object01.runEventHandler(10);\n");

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

        Class01 object01;
        Class01 object02;
        Class1730a obj{ object01, object02 };
        print(PRI1, "main(): async_ltask<int> a = obj.coroutine1();\n");
        async_ltask<int> a = obj.coroutine1();
        print(PRI1, "main(): a.start();\n");
        a.start();

        print(PRI1, "main(): completionflow(obj);\n");
        completionflow(obj);

        print(PRI1, "main(): int v = a.get_result();\n");
        int v = a.get_result();
        print(PRI1, "main(): v = %d\n", v);

        print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(0));
    }

    print(PRI1, "main(): return 0;\n");
    return 0;
}
