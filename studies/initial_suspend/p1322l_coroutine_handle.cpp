/**
 * @file p1322l_coroutine_handle.cpp
 * @brief
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "task_coroutine_handle.h"
#include "mini_awaiter0.h"

mini_awaiter are1;

class Class
{
public:
    task coroutine4()
    {
        printf("coroutine4(): co_await are1;;\n");
        co_await are1;
        printf("coroutine4(): co_return 0;\n");
        co_return 0;
    }

    task coroutine3()
    {
        printf("coroutine3(): co_await coroutine4();\n");
        co_await coroutine4();
        printf("coroutine3(): co_return 0;\n");
        co_return 0;
    }

    task coroutine2()
    {
        printf("coroutine2(): task t = coroutine3();\n");
        task t = coroutine3();
        //int v = co_await t();
        printf("coroutine2(): co_return 0;\n");
        co_return 0;
    }

    task coroutine1()
    {
        printf("coroutine1(): int v = co_await coroutine2();\n");
        co_await coroutine2();
        printf("coroutine1(): co_return 0;\n");
        co_return 0;
    }
};

int main()
{
    Class obj;

    printf("main(): obj.coroutine1().start();\n");
    obj.coroutine1().start();

    printf("main(): std::this_thread::sleep_for(std::chrono::milliseconds(10));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    printf("main(): are1.resume();\n");
    tracker1_obj.nr_resumptions++;
    are1.resume();

    printf("main(): return 0;\n");
    return 0;
}
