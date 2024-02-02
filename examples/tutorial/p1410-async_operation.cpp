/**
 * @file p1410-async_operation.cpp
 * @author
 *
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "p1410.H"

void completionflow(Class1410& obj1410)
{
    // Begin manual event completion
    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "completionflow(): before obj1410.m_object01.eventHandler(10);\n");
    obj1410.m_object01.eventHandler(10);
    print(PRI1, "completionflow(): after obj1410.m_object01.eventHandler(10);\n");

    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "completionflow(): before obj1410.m_object01.eventHandler(10);\n");
    obj1410.m_object01.eventHandler(10);
    print(PRI1, "completionflow(): after obj1410.m_object01.eventHandler(10);\n");
    // End manual event completion
}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    Class01 object01;
    Class1410 obj1410{ object01 };
    async_task<int> a = obj1410.coroutine1();

    print(PRI1, "main(): completionflow(obj1410);\n");
    completionflow(obj1410);

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): return 0;\n");
    return 0;
}
