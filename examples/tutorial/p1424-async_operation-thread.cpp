/**
 * @file p1424-async_operation-thread.cpp
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

    for (int i = 0; i < 10; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        std::mutex mtx;
        ThreadAwaker awaker;
        Class01 object01(UseMode::USE_THREAD, nullptr, nullptr, &mtx, &awaker);
        Class01 object02(UseMode::USE_THREAD, nullptr, nullptr, &mtx, &awaker);
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
    * The following can show a segmmetation fault -> loop counter set to 0
    * Example
      03: Class01::eventHandler[0x7ffc8fd91de0, 0x5583efd1b270](10)
      03: Class01::eventHandler[0x7ffc8fd91de0, 0x5583efd1b270](10): om_async_operation_t->set_result(10);
      00: coroutine5(): co_await wa;
      00: coroutine5(): int v = op1.get_result() + op2.get_result();
      00: coroutine5(): co_return v+1 = 21;
      00: coroutine4(): before v = co_await a;
      Segmentation fault
    */
    for (int i = 0; i < 0; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        std::mutex mtx;
        Class01 object01(UseMode::USE_THREAD, nullptr, nullptr, &mtx, nullptr);
        Class01 object02(UseMode::USE_THREAD, nullptr, nullptr, &mtx, nullptr);
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

    for (int i = 0; i < 0; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        ThreadAwaker awaker;
        Class01 object01(UseMode::USE_THREAD, nullptr, nullptr, nullptr, &awaker);
        Class01 object02(UseMode::USE_THREAD, nullptr, nullptr, nullptr, &awaker);
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

    for (int i = 0; i < 0; ++i)
    {
        print(PRI1, "main(): ---------- iteration %d ----------\n", i);

        Class01 object01(UseMode::USE_THREAD, nullptr, nullptr, nullptr, nullptr);
        Class01 object02(UseMode::USE_THREAD, nullptr, nullptr, nullptr, nullptr);
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
