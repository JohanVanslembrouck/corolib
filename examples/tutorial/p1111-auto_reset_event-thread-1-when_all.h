/**
 * @file p1111-auto_reset_event-thread-1-when_all.h
 * @brief
 * Uses 1 auto_reset_event object that will be resumed from a thread.
 * Coroutine coroutine4 is co_awaited in coroutine3 using when_all.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1111_H_
#define _P1111_H_

#include <thread>

#include <corolib/print.h>
#include <corolib/auto_reset_event.h>
#include <corolib/async_task.h>
#include <corolib/when_all.h>
#include <corolib/threadawaker.h>

using namespace corolib;

class Class1111
{
public:
    Class1111(ThreadAwaker* awaker = nullptr, int delay = 10)
        : m_awaker(awaker)
        , m_delay(delay)
    {
    }

    async_task<int> coroutine4()
    {
        auto_reset_event are;

        if (m_awaker)
            m_awaker->addThread();

        std::thread thread1([this, &are]() {
            print(PRI1, "coroutine4(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", m_delay);
            std::this_thread::sleep_for(std::chrono::milliseconds(m_delay));

            print(PRI1, "coroutine4(): thread1: if (m_awaker) m_awaker->awaitRelease();\n");
            if (m_awaker)
                m_awaker->awaitRelease();

            print(PRI1, "coroutine4(): thread1: are.resume();\n");
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
        print(PRI1, "coroutine3(): int v = co_await coroutine4();\n");
        async_task<int> a = coroutine4();
        print(PRI1, "coroutine3(): when_all wa(a);\n");
        when_all wa(a);
        print(PRI1, "coroutine3(): co_await wa;\n");
        co_await wa;
        print(PRI1, "coroutine3(): int v = a.get_result();\n");
        int v = a.get_result();
        print(PRI1, "coroutine3(): co_return v+1 = %d;\n", v + 1);
        co_return v + 1;
    }

    async_task<int> coroutine2()
    {
        print(PRI1, "coroutine2(): int v = co_await coroutine3();\n");
        int v = co_await coroutine3();
        print(PRI1, "coroutine2(): co_return v+1 = %d;\n", v+1);
        co_return v+1;
    }

    async_task<int> coroutine1()
    {
        print(PRI1, "coroutine1(): int v = co_await coroutine2();\n");
        int v = co_await coroutine2();
        print(PRI1, "coroutine1(): co_return v+1 = %d;\n", v+1);
        co_return v+1;
    }

private:
    ThreadAwaker* m_awaker;
    int m_delay;
};

#endif
