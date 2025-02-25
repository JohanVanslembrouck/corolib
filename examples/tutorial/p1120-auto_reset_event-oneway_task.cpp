/**
 * @file p1120-auto_reset_event-oneway_task.cpp
 * @brief
 * See header file.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include "p1120-auto_reset_event-oneway_task.h"

using namespace corolib;

auto_reset_event m1, m1a, m2, m2a;

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        Class1120 obj;
        print(PRI1, "main(): async_task<int> a1 = obj.coroutine1();\n");
        async_task<int> a1 = obj.coroutine1();

        print(PRI1); print(PRI1, "main(): m2.resume();\n");
        m2.resume();
        print(PRI1); print(PRI1, "main(): m1.resume();\n");
        m1.resume();
        print(PRI1); print(PRI1, "main(): m1.resume();\n");
        m1.resume();
        print(PRI1); print(PRI1, "main(): m2.resume();\n");
        m2.resume();

        print(PRI1); print(PRI1, "main(): int v = a1.get_result();\n");
        int v = a1.get_result();
        print(PRI1, "main(): v = %d\n", v);
    }

    print(PRI1, "main(): return 0;\n");
    return 0;
}
