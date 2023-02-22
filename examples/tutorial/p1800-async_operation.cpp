/**
 * @file p1800-async_operation.cpp
 * @brief
 * Starts an asynchronous operation that will be completed from the main() function.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <functional>

#include "p1800.h"

using namespace corolib;

void completionflow()
{
    print(PRI1, "completionflow()\n");
    // Begin manual event completion

    for (int i = 0; i < 4; i++)
    {
        print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        print(PRI1, "completionflow(): before op.set_result_and_complete(10);\n");
        op.set_result_and_complete(10);
        print(PRI1, "completionflow(): afterop.set_result_and_complete(10)\n");
    }

    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Make coroutine1 co_return
    print(PRI1, "completionflow(): before op.set_result_and_complete(0);\n");
    op.set_result_and_complete(0);
    print(PRI1, "completionflow(): after op.set_result_and_complete(0);\n");
    // End manual event completion
}

int main()
{
    useMode = UseMode::USE_NONE;

    set_priority(0x01);        // Use 0x03 to follow the flow in corolib

    print(PRI1, "main(): async_ltask<int> a = coroutine1();\n");
    async_task<int> a = coroutine1();
 
    print(PRI1, "main(): completionflow();\n");
    completionflow();
    
    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): return 0;\n");
    return 0;
}
