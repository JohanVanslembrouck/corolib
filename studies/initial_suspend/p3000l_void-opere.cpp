/**
 * @file p3000l_void-opere.cpp
 * @brief
 * Uses eager start operations.
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include "task_void.h"

#include "p3000_async_api_opere.h"

task coroutine1()
{
    char buffer[100];
    print(PRI1, "coroutine1: async_oper op = start_write(buffer);\n");
    async_oper op = start_write(buffer);
    print(PRI1, "coroutine1: int res = co_await op;\n");
    int res = co_await op;
    print(PRI1, "coroutine1: res = %d;\n", res);
    print(PRI1, "coroutine1: co_return;\n");
    co_return;
}

// is translated into ...

std::coroutine_handle<> handle;

task coroutine1_tr()
{
    char buffer[100];
    // ...
    async_oper op = start_write(buffer);   // Start running here, see below
    if (!op.await_ready())                      // potential concurrency problem
        op.await_suspend(handle);               // potential concurrency problem
    int res = op.await_resume();                // potential concurrency problem with return valuu
    (void)res;
    // ...
    co_return;
}

task coroutine2()
{
    int res = 0;

    print(PRI1, "coroutine2: async_oper op = start_create();\n");
    async_oper op1 = start_create();
    print(PRI1, "coroutine1: res = co_await op1;\n");
    res = co_await op1;
    print(PRI1, "coroutine2: res = %d;\n", res);

    print(PRI1, "coroutine2: async_oper op2 = start_open();\n");
    async_oper op2 = start_open();
    print(PRI1, "coroutine1: res = co_await op2;\n");
    res = co_await op2;
    print(PRI1, "coroutine2: res = %d;\n", res);

    char buffer[100];
    print(PRI1, "coroutine2: async_oper op3 = start_write(buffer);\n");
    async_oper op3 = start_write(buffer);
    print(PRI1, "coroutine2: int res = co_await op3;\n");
    res = co_await op3;
    print(PRI1, "coroutine2: res = %d;\n", res);

    print(PRI1, "coroutine2: async_oper op4 = start_read();\n");
    async_oper op4 = start_read();
    print(PRI1, "coroutine1: res = co_await op4;\n");
    res = co_await op4;
    print(PRI1, "coroutine2: res = %d;\n", res);

    print(PRI1, "coroutine2: async_oper op5 = start_close();\n");
    async_oper op5 = start_close();
    print(PRI1, "coroutine1: res = co_await op5;\n");
    res = co_await op5;
    print(PRI1, "coroutine2: res = %d;\n", res);

    print(PRI1, "coroutine2: async_oper op6 = start_close();\n");
    async_oper op6 = start_remove();
    print(PRI1, "coroutine1: res = co_await op6;\n");
    res = co_await op6;
    print(PRI1, "coroutine2: res = %d;\n", res);

    print(PRI1, "coroutine2: co_return;\n");
    co_return;
}

EventQueueThrFunctionVoidVoid evqueuethr;
async_oper_base* async_oper_bases[32];
int start_index = 0;

int main()
{
    print(PRI1, "main: task t = coroutine1();\n");
    task t = coroutine1();
    print(PRI1, "main: t.start();\n");
    t.start();
    print(PRI1, "main: runEventQueue(evqueuethr, 1);\n");
    runEventQueue(evqueuethr, 1);

    print(PRI1, "main: task t = coroutine2();\n");
    task t2 = coroutine2();
    print(PRI1, "main: t2.start();\n");
    t2.start();
    print(PRI1, "main: runEventQueue(evqueuethr, 6);\n");
    runEventQueue(evqueuethr, 6);

    print(PRI1, "main: return 0;\n");
    return 0;
}
