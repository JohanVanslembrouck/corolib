/**
 * @file p1706-async_operation-immediate.cpp
 * @brief
 * Starts an asynchronous operation that will be completed from the main() function.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <functional>
#include <thread>

#include "p1700.h"

using namespace corolib;

extern std::function<void(int)> eventHandler;       // p1700.cpp

void completionflow()
{
    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));
}

int main()
{
    useMode = UseMode::USE_IMMEDIATE_COMPLETION;

    set_priority(0x01);        // Use 0x03 to follow the flow in corolib

    {
        print(PRI1, "main(): async_ltask<int> a = coroutine1();\n");
        async_ltask<int> a = coroutine1();
        print(PRI1, "main(): a.start();\n");
        a.start();

        print(PRI1, "main(): completionflow();\n");
        completionflow();

        print(PRI1, "main(): int v = a.get_result();\n");
        int v = a.get_result();
        print(PRI1, "main(): v = %d\n", v);

        print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(0));
    }
    print(PRI1);
	reinit();
    {
        print(PRI1, "main(): async_ltask<void> a = coroutine0();\n");
        async_ltask<void> a = coroutine0();
        print(PRI1, "main(): a.start();\n");
        a.start();

        print(PRI1, "main(): completionflow();\n");
        completionflow();

        print(PRI1, "main(): a.wait();\n");
        a.wait();

        print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(0));
    }

    print(PRI1, "main(): return 0;\n");
    return 0;
}