/**
 * @file p1702-async_operation.cpp
 * @brief
 * Starts an asynchronous operation that will be completed from the main() function.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <functional>

#include "p1700.h"
#include "eventqueue.h"

using namespace corolib;

UseMode useMode = USE_NONE;

extern std::function<void(int)> eventHandler;       // p1700.cpp

async_ltask<void> coroutine0()
{
    async_ltask<int> a = coroutine1();
    co_await a;
}

int main()
{
    set_priority(0x01);        // Use 0x03 to follow the flow in corolib

    print(PRI1, "main(): async_ltask<int> a = coroutine1();\n");
    async_ltask<void> a = coroutine0();
    print(PRI1, "main(): a.start();\n");
    a.start();
    
    // Begin manual event completion
    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): before eventHandler(10);\n");
    eventHandler(10);
    print(PRI1, "main(): after eventHandler(10);\n");

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): before eventHandler(10);\n");
    eventHandler(10);
    print(PRI1, "main(): after eventHandler(10);\n");

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // End manual event completion
    
    print(PRI1, "main(): a.wait();\n");
    a.wait();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): return 0;\n");
    return 0;
}