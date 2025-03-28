/**
 * @file p3000l_void-operl.cpp
 * @brief
 * Uses lazy start operations.
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include "task_void.h"

#include "p3000_async_api_operl.h"

task coroutine1()
{
    print(PRI1, "coroutine1: async_write_oper op;\n");
    async_write_oper op;
    print(PRI1, "coroutine1: int res = co_await op;\n");
    int res = co_await op;
    print(PRI1, "coroutine1: res = %d;\n", res);
    print(PRI1, "coroutine1: co_return;\n");
    co_return;
}

// is translated into...

std::coroutine_handle<> handle;

task coroutine1_tr()
{
    // ...
    async_write_oper op;                  // Does not start here
    if (!op.await_ready())              // always returns false
        op.await_suspend(handle);       // operation will start here
    int res = op.await_resume();        // and may be finished when we come here
    (void)res;
    // ...
    co_return;
}

task coroutine2()
{
    int res = 0;

    print(PRI1, "coroutine2: async_create_oper op1;\n");
    async_create_oper op1;
    print(PRI1, "coroutine2: res = co_await op1;\n");
    res = co_await op1;
    print(PRI1, "coroutine2: res = %d;\n", res);

    print(PRI1, "coroutine2: async_open_oper op2;\n");
    async_open_oper op2;
    print(PRI1, "coroutine2: res = co_await op2;\n");
    res = co_await op2;
    print(PRI1, "coroutine2: res = %d;\n", res);

    print(PRI1, "coroutine2: async_write_oper op3;\n");
    async_write_oper op3;
    print(PRI1, "coroutine2: int res = co_await op3;\n");
    res = co_await op3;
    print(PRI1, "coroutine2: res = %d;\n", res);

    print(PRI1, "coroutine2: async_read_oper op4;\n");
    async_create_oper op4;
    print(PRI1, "coroutine2: res = co_await op4;\n");
    res = co_await op4;
    print(PRI1, "coroutine2: res = %d;\n", res);

    print(PRI1, "coroutine2: async_close_oper op5;\n");
    async_close_oper op5;
    print(PRI1, "coroutine2: res = co_await op5;\n");
    res = co_await op5;
    print(PRI1, "coroutine2: res = %d;\n", res);
    
    print(PRI1, "coroutine2: async_remove_oper op6;\n");
    async_remove_oper op6;
    print(PRI1, "coroutine2: res = co_await op6;\n");
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
