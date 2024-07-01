/**
 * @file class_async-thread.h
 * @brief
 * Class with 4 coroutines and asynchronous completion in the last-called coroutine.
 * Completion is run on a dedicated thread.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _CLASS_ASYNC_THREAD_H_
#define _CLASS_ASYNC_THREAD_H_

#include <coroutine>

#include "mini_awaiter.h"
#include "print.h"

class Class
{
public:
    task coroutine4()
    {
        mini_awaiter are1;

        std::thread thread1([this, &are1]() {
            print(PRI1, "coroutine4(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", 1000);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            print(PRI1, "coroutine4(): thread1: are1.resume();\n");
            are1.resume();
            print(PRI1, "coroutine4(): thread1: return;\n");
            });
        thread1.detach();

        print(PRI1, "coroutine4(): co_await are1;\n");
        co_await are1;
        print(PRI1, "coroutine4(): co_return 1;\n");
        co_return 1;
    }

    task coroutine3()
    {
        print(PRI1, "coroutine3(): int v = co_await coroutine4();\n");
        int v = co_await coroutine4();
        print(PRI1, "coroutine3(): co_return %d;\n", v + 1);
        co_return v + 1;
    }

    task coroutine2()
    {
        print(PRI1, "coroutine2(): int v = co_await coroutine3();\n");
        int v = co_await coroutine3();
        print(PRI1, "coroutine2(): co_return %d;\n", v + 1);
        co_return v + 1;
    }

    task coroutine1()
    {
        print(PRI1, "coroutine1(): int v = co_await coroutine2();\n");
        int v = co_await coroutine2();
        print(PRI1, "coroutine1(): co_return %d;\n", v + 1);
        co_return v + 1;
    }
};

#endif
