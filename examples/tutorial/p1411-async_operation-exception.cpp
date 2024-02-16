/**
 * @file p1411-async_operation-exception.cpp
 * @author
 *
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "p1410.h"

void completionflow(Class1410& obj)
{
    // Begin manual event completion
    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    try {
        print(PRI1, "completionflow(): before obj.m_object01.runEventHandler(-1);\n");
        obj.m_object01.runEventHandler(-1);
        print(PRI1, "completionflow(): after obj.m_object01.runEventHandler(-1);\n");
    }
    catch (...) {
        print(PRI1, "completionflow: caught exception after object01.runEventHandler(-1);\n");
    }

    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "completionflow(): before object01.runEventHandler(10);\n");
    obj.m_object01.runEventHandler(10);
    print(PRI1, "completionflow(): after object01.runEventHandler(10);\n");
    // End manual event completion
}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    Class01 object01;
    Class1410 obj{ object01 };
    async_task<int> a = obj.coroutine1();

    print(PRI1, "main(): completionflow(obj);\n");
    completionflow(obj);

    print(PRI1, "main(): int v = a.get_result(false);\n");
    int v = a.get_result(false);
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): return 0;\n");
    return 0;
}
