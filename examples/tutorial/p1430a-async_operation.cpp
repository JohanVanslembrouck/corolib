/**
 * @file p1430a-async_operation.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "p1430a.h"

void completionflow(Class1430a& obj1430a)
{
    // Begin manual event completion
    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "completionflow(): before obj1430a.m_object01.eventHandler(10);\n");
    obj1430a.m_object01.eventHandler(10);
    print(PRI1, "completionflow(): after obj1430a.m_object01.eventHandler(10);\n");

    print(PRI1, "completionflow(): before obj1430a.m_object02.eventHandler(10);\n");
    obj1430a.m_object02.eventHandler(10);
    print(PRI1, "completionflow(): after obj1430a.m_object02.eventHandler(10);\n");

    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "completionflow(): before obj1430a.m_object02.eventHandler(10);\n");
    obj1430a.m_object02.eventHandler(10);
    print(PRI2, "completionflow(): after obj1430a.m_object02.eventHandler(10);\n");

    print(PRI1, "completionflow(): before obj1430a.m_object01.eventHandler(10);\n");
    obj1430a.m_object01.eventHandler(10);
    print(PRI1, "completionflow(): after obj1430a.m_object01.eventHandler(10);\n");

    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // End manual event completion
}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    Class01 object01;
    Class01 object02;
    Class1430a obj1430a{ object01, object02 };
    async_task<int> a = obj1430a.coroutine1();

    print(PRI1, "main(): completionflow(obj1430a);\n");
    completionflow(obj1430a);

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): return 0;\n");
    return 0;
}