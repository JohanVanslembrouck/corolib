/**
 * @file p1105-auto_reset_event-2-when_any.cpp
 * @brief
 * See header file.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <thread>

#include "p1105-auto_reset_event-2-when_any.h"

using namespace corolib;

auto_reset_event are1;
auto_reset_event are2;

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        Class1105 obj;
        print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
        async_task<int> a = obj.coroutine1();

        print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        print(PRI1, "main(): are1.resume();\n");
        are1.resume();

        print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        print(PRI1, "main(): are2.resume();\n");
        are2.resume();

        print(PRI1, "main(): int v = a.get_result();\n");
        int v = a.get_result();
        print(PRI1, "main(): v = %d\n", v);
    }

    print(PRI1, "main(): return 0;\n");
    return 0;
}
