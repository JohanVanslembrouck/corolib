/**
 * @file p1610-async_operation.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "p1610.h"

void completionflow(Class1490& obj)
{
    // Begin manual event completion
    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));

    // Complete operation1 call in 1st coroutine2 instance
    print(PRI1);
    print(PRI1, "completionflow(): before obj.m_object.runEventHandler(%d, 10);\n", 0);
    obj.m_object.runEventHandler(0, 10);
    print(PRI1, "completionflow(): after obj.m_object.runEventHandler(%d, 10);\n", 0);

    // Complete operation1 call in 2nd coroutine2 instance
    print(PRI1);
    print(PRI1, "completionflow(): before obj.m_object.runEventHandler(%d, 10);\n", 1);
    obj.m_object.runEventHandler(1, 10);
    print(PRI1, "completionflow(): after ob.m_object.runEventHandler(%d, 10);\n", 1);

    // Complete operation2 call in 1st coroutine2 instance
    print(PRI1);
    print(PRI1, "completionflow(): before obj.m_object.runEventHandler(%d, 20);\n", 3);
    obj.m_object.runEventHandler(3, 20);
    print(PRI1, "completionflow(): after obj.m_object.runEventHandler(%d, 20);\n", 3);

    // Complete operation1 call in 3rd coroutine2 instance
    print(PRI1);
    print(PRI1, "completionflow(): before obj.m_object.runEventHandler(%d, 10);\n", 2);
    obj.m_object.runEventHandler(2, 10);
    print(PRI1, "completionflow(): after obj.m_object.runEventHandler(%d, 10);\n", 2);

    print(PRI1);
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

        Class02 object;
        Class1490 obj{ object };

        print(PRI1, "main(): async_task<int> a1 = obj.coroutine1(10);\n");
        async_task<int> a1 = obj.coroutine1(10);

        print(PRI1, "main(): async_task<int> a2 = obj.coroutine1(20);\n");
        async_task<int> a2 = obj.coroutine1(20);

        print(PRI1, "main(): async_task<int> a2 = obj.coroutine1(30);\n");
        async_task<int> a3 = obj.coroutine1(30);

        print(PRI1, "main(): completionflow(obj);\n");
        completionflow(obj);

        print(PRI1, "main(): int v1 = a1.get_result();\n");
        int v1 = a1.get_result();
        print(PRI1, "main(): v1 = %d\n", v1);

        print(PRI1, "main(): int v2 = a2.get_result();\n");
        int v2 = a2.get_result();
        print(PRI1, "main(): v2 = %d\n", v2);

        print(PRI1, "main(): int v3 = a2.get_result();\n");
        int v3 = a3.get_result();
        print(PRI1, "main(): v3 = %d\n", v3);

        print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(0));
    }

    print(PRI1, "main(): return 0;\n");
    return 0;
}
