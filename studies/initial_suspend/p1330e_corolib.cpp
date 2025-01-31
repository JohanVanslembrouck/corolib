/**
 * @file p1330e_corolib.cpp
 * @brief
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <corolib/print.h>
#include <corolib/async_task.h>

#include "mini_awaiter0.h"

using namespace corolib;

mini_awaiter are1;

class Class
{
public:
    async_task<int> coroutine4()
    {
        print(PRI1, "coroutine4(): co_await are1;\n");
        co_await are1;
        print(PRI1, "coroutine4(): co_return 1;\n");
        co_return 1;
    }

    async_task<int> coroutine3()
    {
        print(PRI1, "coroutine3(): async_task<int> t = coroutine4();\n");
        async_task<int> t = coroutine4();
        print(PRI1, "coroutine3(): int v = co_await t;\n");
        int v = co_await t;
        print(PRI1, "coroutine3(): co_return v+1 = %d;\n", v + 1);
        co_return v + 1;
    }

    async_task<int> coroutine2()
    {
        print(PRI1, "coroutine2(): async_task<int> t = coroutine3();\n");
        async_task<int> t = coroutine3();
        print(PRI1, "coroutine2(): int v = co_await t;\n");
        int v = co_await t;
        print(PRI1, "coroutine2(): co_return v+1 = %d;\n", v + 1);
        co_return v + 1;
    }

    async_task<int> coroutine1()
    {
        print(PRI1, "coroutine1(): async_task<int> t = coroutine2();\n");
        async_task<int> t = coroutine2();
        print(PRI1, "coroutine1(): int v = co_await t;\n");
        int v = co_await t;
        print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v + 1);
        co_return v + 1;
    }
};

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    Class obj;
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
	
    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    print(PRI1, "main(): are1.resume();\n");
    are1.resume();
	
    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);
    print(PRI1, "main(): return 0;\n");
    return 0;
}
