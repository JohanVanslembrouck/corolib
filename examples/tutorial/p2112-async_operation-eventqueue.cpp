/**
 * @file p2112-async_operation-eventqueue.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <functional>
#include <future>

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "p2110.h"

void completionflow(CoroClass01& coroObject)
{
    print(PRI1, "completionflow(): coroObject.m_object.getEventQueue()->run();\n");
    coroObject.m_object.getEventQueue()->run();
}

EventQueue eventQueue1;
Class01 object01(USE_EVENTQUEUE, &eventQueue1);
CoroClass01 coroObject01(object01);

EventQueue eventQueue2;
Class01 object02(USE_EVENTQUEUE, &eventQueue2);
CoroClass01 coroObject02(object02);

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    print(PRI1, "main(): auto task1thr = std::async(std::launch::async, task1, coroObject01);\n");
    auto task1thr = std::async(std::launch::async, task1, coroObject01);
    print(PRI1, "main(): auto task2thr = std::async(std::launch::async, task1, coroObject02);\n");
    auto task2thr = std::async(std::launch::async, task1, coroObject02);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): int v = task1thr.get() + task2thr.get();\n");
    int v = task1thr.get() + task2thr.get();

    print(PRI1, "main(): v = %d;\n", v);
    print(PRI1, "main(): return 0;\n");
    return 0;
}
