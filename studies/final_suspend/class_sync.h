/**
 * @file class_sync.h
 * @brief
 * Class with 4 coroutines and synchronous completion in the last-called coroutine.
 * 
 * This file does not include a definition of class 'task'.
 * A task header file will be included in the pXXXX files in which
 * this file will be included just after the inclusion of this task header file.
 * This allows the same coroutine code to be used with different task definitions.
 * 
 * Execution of the statement
 *      int v = co_await coroutineX();
 * creates a temporary task object that will go out-of-scope
 * at the end of the statement.
 * Task's destructor will thus be called before execution of the co_return statement.
 * 
 * @author Johan Vanslembrouck
 */

#ifndef _CLASS_SYNC_H_
#define _CLASS_SYNC_H_

#include <coroutine>

#include "print.h"

class Class
{
public:
    task coroutine4()
    {
        print(PRI1, "coroutine4(): co_return 1;\n");
        co_return 1;
    }

    task coroutine3()
    {
        print(PRI1, "coroutine3() : int v = co_await coroutine4(); \n");
        int v = co_await coroutine4();
        print(PRI1, "coroutine3() : co_return %d; \n", v + 1);
        co_return v + 1;
    }

    task coroutine2()
    {
        print(PRI1, "coroutine2(): int v = co_await coroutine3();\n");
        int v = co_await coroutine3();
        print(PRI1, "coroutine2() : co_return %d; \n", v + 1);
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
