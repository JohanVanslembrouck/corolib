/**
 * @file class_async4.h
 * @brief
 * Class with 4 coroutines and asynchronous completion in the last-called coroutine.
 *
 * The Class defined in this file is a variant of the Class defined in class_async2.h.
 * 
 * The task object is defined at the function scope instead of at a local scope as in class_async2.h.
 * The task object will go out-of-scope after the co_retun statement instead of before
 * which is the case in class_async.h and class_async2.h.
 * 
 * @author Johan Vanslembrouck
 */

#ifndef _CLASS_ASYNC4_H_
#define _CLASS_ASYNC4_H_

#include <coroutine>

#include "mini_awaiter.h"
#include "print.h"

extern mini_awaiter are1;

class Class
{
public:
    task coroutine4()
    {
        print(PRI1, "coroutine4(): co_await are1;\n");
        co_await are1;
        print(PRI1, "coroutine4(): co_return 1;\n");
        co_return 1;
    }

    task coroutine3()
    {
        print(PRI1, "coroutine3(): task t = coroutine4();\n");
        task t = coroutine4();
        print(PRI1, "coroutine3(): int v = co_await t;\n");
        int v = co_await t;
        print(PRI1, "coroutine3(): co_return %d;\n", v + 1);
        co_return v + 1;
    }

    task coroutine2()
    {
        print(PRI1, "coroutine2(): task t = coroutine3();\n");
        task t = coroutine3();
        print(PRI1, "coroutine2(): int v = co_await t;\n");
        int v = co_await t;
        print(PRI1, "coroutine2(): co_return %d;\n", v + 1);
        co_return v + 1;
    }

    task coroutine1()
    {
        print(PRI1, "coroutine1(): task t = coroutine2();\n");
        task t = coroutine2();
        print(PRI1, "coroutine1(): int v = co_await t;\n");
        int v = co_await t;
        print(PRI1, "coroutine1(): co_return %d;\n", v + 1);
        co_return v + 1;
    }
};

#endif
