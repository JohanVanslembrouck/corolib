/**
 * @file p1429-async_operation-thread-imm.cpp
 * @brief
 *
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

using namespace corolib;

#include "p1420.h"

void completionflow(ThreadAwaker* awaker)
{
    if (awaker)
        awaker->releaseThreads();
}

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    /*
    * The following can show a segmmetation fault
    * Example
      02: Class01::eventHandler[0x7ffc2ae07360, 0x561928863220](10)
      02: Class01::eventHandler[0x7ffc2ae07360, 0x561928863220](10): om_async_operation_t->set_result(10);
      01: coroutine5(): co_await wa;
      01: coroutine5(): int v = op1.get_result() + op2.get_result();
      01: coroutine5(): co_return v+1 = 21;
      01: coroutine4(): before v = co_await a;
      01: coroutine4(): after v = co_await a;
      Segmentation fault
    */
    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        ThreadAwaker awaker;
        Class01 object01(UseMode::USE_THREAD, nullptr, nullptr, nullptr, &awaker);
        Class01 object02(UseMode::USE_IMMEDIATE_COMPLETION);
        Class1420 obj{ object01, object02 };
        async_task<int> a = obj.coroutine1();

        print(PRI1, "main(): completionflow(&awaker);\n");
        completionflow(&awaker);

        print(PRI1, "main(): int v = a.get_result();\n");
        int v = a.get_result();
        print(PRI1, "main(): v = %d\n", v);

        print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(0));
    }

    /*
    * 
    * The following can show a segmmetation fault -> loop counter set to 0
    * Example
      01: Class01::eventHandler[0x7ffe83464390, 0x5635b00b4220](10): om_async_operation_t->set_result(10);
      00: coroutine5(): co_await wa;
      00: coroutine5(): int v = op1.get_result() + op2.get_result();
      00: coroutine5(): co_return v+1 = 21;
      00: coroutine4(): before v = co_await a;
      00: coroutine4(): after v = co_await a;
      00: coroutine4(): co_return v+1 = 22;
      00: coroutine3(): int v = co_await a1;
      Segmentation fault
    */
    for (int i = 0; i < 0; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        Class01 object01(UseMode::USE_THREAD, nullptr, nullptr, nullptr, nullptr);
        Class01 object02(UseMode::USE_IMMEDIATE_COMPLETION);
        Class1420 obj{ object01, object02 };
        async_task<int> a = obj.coroutine1();

        print(PRI1, "main(): completionflow(nullptr);\n");
        completionflow(nullptr);

        print(PRI1, "main(): int v = a.get_result();\n");
        int v = a.get_result();
        print(PRI1, "main(): v = %d\n", v);

        print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(0));
    }

    print(PRI1, "main(): return 0;\n");
    return 0;
}
