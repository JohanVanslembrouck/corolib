/**
 * @file p1118-auto_reset_event-thread-3-when_any.h
 * @brief
 * Uses 3 auto_reset_event objects that will be resumed from 3 threads.
 * Coroutines coroutine4a, coroutine4b and coroutine4c are co_awaited in coroutine3 using when_any.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1118_H_
#define _P1118_H_

#include <thread>
#include <mutex>

#include <corolib/print.h>
#include <corolib/auto_reset_event.h>
#include <corolib/async_task.h>
#include <corolib/when_any.h>
#include <corolib/threadawaker.h>

using namespace corolib;

class Class1118
{
public:
    Class1118(std::mutex* mtx = nullptr, ThreadAwaker* awaker = nullptr, int delay = 10)
        : m_mutex(mtx)
        , m_awaker(awaker)
        , m_delay(delay)
    {
    }

    async_task<int> coroutine4a()
    {
        auto_reset_event are;

        if (m_awaker)
            m_awaker->addThread();

        std::thread thread1([this, &are]() {
            print(PRI1, "coroutine4a(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", m_delay);
            std::this_thread::sleep_for(std::chrono::milliseconds(m_delay));

            print(PRI1, "coroutine4a(): thread1: if (m_awaker) m_awaker->awaitRelease();\n");
            if (m_awaker)
                m_awaker->awaitRelease();

            if (m_mutex) {
                std::lock_guard<std::mutex> guard(*m_mutex);
                print(PRI1, "coroutine4a(): thread1: are.resume();\n");
                are.resume();
            }
            else {
                print(PRI1, "coroutine4a(): thread1: are.resume();\n");
                are.resume();
            }

            print(PRI1, "coroutine4a(): thread1: return;\n");
            });
        thread1.detach();

        print(PRI1, "coroutine4a(): co_await are;\n");
        co_await are;

        print(PRI1, "coroutine4a(): co_return 1;\n");
        co_return 1;
    }

    async_task<int> coroutine4b()
    {
        auto_reset_event are;

        if (m_awaker)
            m_awaker->addThread();

        std::thread thread1([this, &are]() {
            print(PRI1, "coroutine4b(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", m_delay);
            std::this_thread::sleep_for(std::chrono::milliseconds(m_delay));

            print(PRI1, "coroutine4b(): thread1: if (m_awaker) m_awaker->awaitRelease();\n");
            if (m_awaker)
                m_awaker->awaitRelease();

            if (m_mutex) {
                std::lock_guard<std::mutex> guard(*m_mutex);
                print(PRI1, "coroutine4b(): thread1: are.resume();\n");
                are.resume();
            }
            else {
                print(PRI1, "coroutine4b(): thread1: are.resume();\n");
                are.resume();
            }

            print(PRI1, "coroutine4b(): thread1: return;\n");
            });
        thread1.detach();

        print(PRI1, "coroutine4b(): co_await are;\n");
        co_await are;

        print(PRI1, "coroutine4b(): co_return 2;\n");
        co_return 1;
    }

    async_task<int> coroutine4c()
    {
        auto_reset_event are;

        if (m_awaker)
            m_awaker->addThread();

        std::thread thread1([this, &are]() {
            print(PRI1, "coroutine4c(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", m_delay);
            std::this_thread::sleep_for(std::chrono::milliseconds(m_delay));

            print(PRI1, "coroutine4c(): thread1: if (m_awaker) m_awaker->awaitRelease();\n");
            if (m_awaker)
                m_awaker->awaitRelease();

            if (m_mutex) {
                std::lock_guard<std::mutex> guard(*m_mutex);
                print(PRI1, "coroutine4c(): thread1: are.resume();\n");
                are.resume();
            }
            else {
                print(PRI1, "coroutine4c(): thread1: are.resume();\n");
                are.resume();
            }

            print(PRI1, "coroutine4c(): thread1: return;\n");
            });
        thread1.detach();

        print(PRI1, "coroutine4c(): co_await are;\n");
        co_await are;

        print(PRI1, "coroutine4c(): co_return 2;\n");
        co_return 1;
    }

    async_task<int> coroutine3()
    {
        int v1, v2, v3;
        v1 = v2 = v3 = 0;

        print(PRI1, "coroutine3(): async_task<int> a = coroutine4a();\n");
        async_task<int> a = coroutine4a();
        print(PRI1, "coroutine3(): async_task<int> b = coroutine4b();\n");
        async_task<int> b = coroutine4b();
        print(PRI1, "coroutine3(): async_task<int> c = coroutine4c();\n");
        async_task<int> c = coroutine4c();

        print(PRI1, "coroutine3(): when_any wa(a, b, c);\n");
        when_any wa(a, b, c);

        for (int j = 0; j < 3; ++j)
        {
            print(PRI1, "coroutine3(): i = co_await wa;\n");
            int i = co_await wa;
            print(PRI1, "coroutine3(): i = %d\n", i);
            switch (i) {
            case 0:
                print(PRI1, "coroutine3(): v1 = a.get_result();\n");
                v1 = a.get_result();
                break;
            case 1:
                print(PRI1, "coroutine3(): v2 = b.get_result();\n");
                v2 = b.get_result();
                break;
            case 2:
                print(PRI1, "coroutine3(): v3 = c.get_result();\n");
                v3 = c.get_result();
                break;
            default:
                print(PRI1, "coroutine3(): invalid index = %d\n", i);
            }
        }

        print(PRI1, "coroutine3(): co_return v+1 = %d;\n", v1 + v2 + v3 + 1);
        co_return v1 + v2 + v3 + 1;
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
    std::mutex* m_mutex;
    ThreadAwaker* m_awaker;
    int m_delay;
};

#endif
