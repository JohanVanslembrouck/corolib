/**
 * @file p1322l_coroutine_handle-thread.cpp
 * @brief
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "task_coroutine_handle.h"
#include "mini_awaiter0.h"

class Class
{
public:
    Class(int delay = 10)
        : m_delay(delay)
    {
    }

    task coroutine4()
    {
        mini_awaiter are;

        std::thread thread1([this, &are]() {
            printf("coroutine4(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", m_delay);
            std::this_thread::sleep_for(std::chrono::milliseconds(m_delay));

            printf("coroutine4(): thread1: are.resume();\n");
            tracker1_obj.nr_resumptions++;
            are.resume();

            printf("coroutine4(): thread1: return;\n");
            });
        thread1.detach();

        printf("coroutine4(): co_await are;\n");
        co_await are;

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
        printf("coroutine2():  task t = coroutine3();\n");
        task t = coroutine3();
        //int v = co_await coroutine3();
        printf("coroutine2(): co_return;\n");
        co_return;
    }

    task coroutine1()
    {
        printf("coroutine1(): co_await coroutine2();\n");
        co_await coroutine2();
        printf("coroutine1(): co_return;\n");
        co_return;
    }

private:
    int m_delay;
};

int main()
{
    Class obj{ 1000 };
    printf("main(): obj.coroutine1().start();\n");
    obj.coroutine1().start();

    printf("main(): std::this_thread::sleep_for(std::chrono::milliseconds(1100));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));

    printf("main(): return 0;\n");
    return 0;
}
