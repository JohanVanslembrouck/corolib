/**
 * @file p1330e_corolib-thread.cpp
 * @brief
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <thread>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/threadawaker.h>

#include "mini_awaiter.h"
#include "tracker1.h"

using namespace corolib;

class Class
{
public:
    Class(ThreadAwaker* awaker = nullptr, int delay = 10)
        : m_awaker(awaker)
        , m_delay(delay)
    {
    }

    async_task<int> coroutine4()
    {
        mini_awaiter are;

        if (m_awaker)
            m_awaker->addThread();

        std::thread thread1([this, &are]() {
            print(PRI1, "coroutine4(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", m_delay);
            std::this_thread::sleep_for(std::chrono::milliseconds(m_delay));

            print(PRI1, "coroutine4(): thread1: if (m_awaker) m_awaker->awaitRelease();\n");
            if (m_awaker)
                m_awaker->awaitRelease();

            print(PRI1, "coroutine4(): thread1: are.resume();\n");
            tracker1_obj.nr_resumptions++;
            are.resume();

            print(PRI1, "coroutine4(): thread1: return;\n");
            });
        thread1.detach();

        print(PRI1, "coroutine4(): co_await are;\n");
        co_await are;

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

private:
    ThreadAwaker* m_awaker;
    int m_delay;
};

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    ThreadAwaker awaker;
    Class obj{ nullptr, 1000 };
    print(PRI1, "main(): async_task<int> a = obj.coroutine1();\n");
    async_task<int> a = obj.coroutine1();
    print(PRI1, "main(): awaker.releaseThreads();\n");
    awaker.releaseThreads();
    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);
    print(PRI1, "main(): return 0;\n");
    return 0;
}
