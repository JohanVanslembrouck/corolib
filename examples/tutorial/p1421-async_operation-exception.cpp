/**
 * @file p1421-async_operation-exception.cpp
 * @brief
 *
 * 
 * @author Johan Vanslembrouck
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "p1420.h"

void completionflow(Class1420& obj)
{
    // Begin manual event completion
    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));

    print(PRI1, "completionflow(): 1a. before object02.runEventHandler(10);\n");
    obj.m_object02.runEventHandler(10);
    print(PRI1, "completionflow(): 1a. after object02.runEventHandler(10);\n");
;
    print(PRI1, "completionflow(): 1b. before object01.runEventHandler(-1);\n");
    obj.m_object01.runEventHandler(-1);
    print(PRI1, "completionflow(): 1b. after object01.runEventHandler(-1);\n");

    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));

    print(PRI1, "completionflow(): 2a. before object01.runEventHandler(10);\n");
    obj.m_object01.runEventHandler(10);
    print(PRI1, "completionflow(): 2a. after object01.runEventHandler(10);\n");

    print(PRI1, "completionflow(): 2b. before object02.runEventHandler(10);\n");
    obj.m_object02.runEventHandler(10);
    print(PRI1, "completionflow(): 2b. after object02.runEventHandler(10);\n");

    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));
    // End manual event completion
}

int main()
{
    set_priority(0x01);        // Use 0x03 to follow the flow in corolib

    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        Class01 object01;
        Class01 object02;
        Class1420 obj{ object01, object02 };
        async_task<int> a = obj.coroutine1();

        print(PRI1, "main(): completionflow(obj1410);\n");
        completionflow(obj);

        print(PRI1, "main(): int v = a.get_result(false);\n");
        int v = a.get_result(false);
        print(PRI1, "main(): v = %d\n", v);

        print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(0));
    }

    print(PRI1, "main(): return 0;\n");
    return 0;
}
