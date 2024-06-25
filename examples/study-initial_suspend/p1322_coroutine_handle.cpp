/**
 * @file p1322e_coroutine_handle.cpp
 * @brief
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "taske_coroutine_handle.h"
#include "mini_awaiter.h"

mini_awaiter are1;

class Class
{
public:
    task coroutine4()
    {
        printf("coroutine4(): co_await are1;\n");
        co_await are1;
        printf("coroutine4(): co_return;\n");
        co_return;
    }

    task coroutine3()
    {
        printf("coroutine3(): co_await coroutine4();\n");
        co_await coroutine4();
        printf("coroutine3(): co_return;\n");
        co_return;
    }

    task coroutine2()
    {
        printf("coroutine2(): task t = coroutine3();\n");
        task t = coroutine3();
        //int v = co_await t();
        printf("coroutine2(): co_return;\n");
        co_return;
    }

    task coroutine1()
    {
        printf("coroutine1(): int v = co_await coroutine2();\n");
        co_await coroutine2();
        printf("coroutine1(): co_return;\n");
        co_return;
    }
};

int main()
{
    Class obj;
    printf("main(): task a = obj.coroutine1();\n");
    task a = obj.coroutine1();
	
    printf("main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    printf("main(): are1.resume();\n");
    are1.resume();
	
    printf("main(): return 0;\n");
    return 0;
}
