/**
 * @file p1112-auto_reset_event-thread-1-when_any.cpp
 * @brief
 * See header file.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "p1112-auto_reset_event-thread-1-when_any.h"

using namespace corolib;

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        ThreadAwaker awaker;
        Class1112 obj{ &awaker, 0 };
        print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
        async_task<int> a = obj.coroutine1();
        print(PRI1, "main(): awaker.releaseThreads();\n");
        awaker.releaseThreads();
        print(PRI1, "main(): int v = a.get_result();\n");
        int v = a.get_result();
        print(PRI1, "main(): v = %d\n", v);
    }

    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        Class1112 obj{ nullptr, 0 };
        print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
        async_task<int> a = obj.coroutine1();
        print(PRI1, "main(): int v = a.get_result();\n");
        int v = a.get_result();
        print(PRI1, "main(): v = %d\n", v);
    }

    print(PRI1, "main(): return 0;\n");
    return 0;
}
