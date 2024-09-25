/**
 * @file p1320_coroutine_handle.cpp
 * @brief
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "task_coroutine_handle.h"
#include "sync_wait_task.h"
#include "manual_executor.h"

manual_executor ex;

class Class
{
public:
    task coroutine4()
    {
        printf("coroutine4(): co_await ex.schedule();\n");
        co_await ex.schedule();
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
        printf("coroutine2(): co_await coroutine3();\n");
        co_await coroutine3();
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
   
    printf("main(): ex.sync_wait(obj.coroutine1());\n");
    ex.sync_wait(obj.coroutine1());
	
    printf("main(): return 0;\n");
    return 0;
}
