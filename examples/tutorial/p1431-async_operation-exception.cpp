/**
 * @file p1430-async_operation.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "p1430.h"

void completionflow(Class1430& obj)
{
    // Begin manual event completion
    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));

    print(PRI1, "completionflow(): before obj.m_object01.runEventHandler(-1);\n");
    obj.m_object01.runEventHandler(-1);
    print(PRI1, "completionflow(): after obj.m_object01.runEventHandler(-1);\n");

    print(PRI1, "completionflow(): before obj.m_object02.runEventHandler(10);\n");
    obj.m_object02.runEventHandler(10);
    print(PRI1, "completionflow(): after obj.m_object02.runEventHandler(10);\n");

    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));

    print(PRI1, "completionflow(): before obj.m_object02.runEventHandler(10);\n");
    obj.m_object02.runEventHandler(10);
    print(PRI2, "completionflow(): after obj.m_object02.runEventHandler(10);\n");

    print(PRI1, "completionflow(): before obj.m_object01.runEventHandler(10);\n");
    obj.m_object01.runEventHandler(10);
    print(PRI1, "completionflow(): after obj.m_object01.runEventHandler(10);\n");

    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));
    // End manual event completion
}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    /**
     *
     * The following hangs:
     * Example:
       00: completionflow(): after obj.m_object02.runEventHandler(10);
       00: completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(0));
       00: completionflow(): before obj.m_object02.runEventHandler(10);
       00: Class01::eventHandler[000000AD74B3F0C0, 0000026201BFC6C0](10)
       00: Class01::eventHandler[000000AD74B3F0C0, 0000026201BFC6C0](10): om_async_operation_t->set_result(10);
       00: completionflow(): before obj.m_object01.runEventHandler(10);
       00: Class01::eventHandler[000000AD74B3E820, 0000026201BFC4F0](10)
       00: Class01::eventHandler[000000AD74B3E820, 0000026201BFC4F0](10): om_async_operation_t->set_result(10);
       00: completionflow(): after obj.m_object01.runEventHandler(10);
       00: completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(0));
       00: main(): int v = a.get_result();
    */
    for (int i = 0; i < 1; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        Class01 object01;
        Class01 object02;
        Class1430 obj{ object01, object02 };
        async_task<int> a = obj.coroutine1();

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
