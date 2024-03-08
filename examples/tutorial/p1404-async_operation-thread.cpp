/**
 * @file p1404-async_operation-thread.cpp
 * @brief
 * Starts an asynchronous operation that will be completed after one second from a thread.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <thread>

#include "p1400.h"
#include "eventqueue.h"

using namespace corolib;

void completionflow()
{
    if (awaker)
        awaker->releaseThreads();
}

int main()
{
    useMode = UseMode::USE_THREAD;

    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    ThreadAwaker awaker_;
    awaker = &awaker_;

    print(PRI1, "main(): async_task<int> a = coroutine1();\n");
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
