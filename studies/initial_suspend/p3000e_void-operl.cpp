/**
 * @file p3000e_void-operl.cpp
 * @brief
 * Uses eager start tasks and lazy start operations.
 * 
 * @author Johan Vanslembrouck
 */

#include "taske_void_p.h"

#include "p3000_async_api_operl.h"

task coroutine1()
{
    char buffer[100];
    sprintf(buffer, "Input string\n");
    print(PRI1, "coroutine1(): async_write_oper op = start_write();\n");
    async_write_oper op = start_write(buffer);
    print(PRI1, "coroutine1(): int res = co_await op;\n");
    int res = co_await op;
    print(PRI1, "coroutine1(): res = %d\n", res);
    print(PRI1, "coroutine1(): co_return 0;\n");
    co_return 0;
}

// is translated into...

std::coroutine_handle<> handle;

task coroutine1_tr()
{
    char buffer[100];
    // ...
    async_write_oper op = start_write(buffer);  // Does not start here
    if (!op.await_ready())                      // always returns false
        op.await_suspend(handle);               // operation will start here
    int res = op.await_resume();                // and may be finished when we come here
    (void)res;
    // ...
    co_return 0;
}

task coroutine2()
{
    print(PRI1, "coroutine2(): async_create_oper op1 = start_create();\n");
    async_create_oper op1 = start_create();
    print(PRI1, "coroutine2(): bool res1 = co_await op1;\n");
    bool res1 = co_await op1;
    print(PRI1, "coroutine2(): res1 = %d\n", res1);

    print(PRI1, "coroutine2(): async_open_oper op2 = start_open();\n");
    async_open_oper op2 = start_open();
    print(PRI1, "coroutine2(): int res2 = co_await op2;\n");
    int res2 = co_await op2;
    print(PRI1, "coroutine2(): res2 = %d\n", res2);

    char buffer[100];
    sprintf(buffer, "Input string\n");
    print(PRI1, "coroutine2(): async_write_oper op3 = start_write(buffer);\n");
    async_write_oper op3 = start_write(buffer);
    print(PRI1, "coroutine2(): int res3 = co_await op3;\n");
    int res3 = co_await op3;
    print(PRI1, "coroutine2(): res3 = %d\n", res3);

    print(PRI1, "coroutine2(): async_read_oper op4 = start_read();\n");
    async_read_oper op4 = start_read();
    print(PRI1, "coroutine2(): std::string res4 = co_await op4;\n");
    std::string res4 = co_await op4;
    print(PRI1, "coroutine2(): res4 = %s\n", res4.c_str());

    print(PRI1, "coroutine2(): async_close_oper op5 = start_close();\n");
    async_close_oper op5 = start_close();
    print(PRI1, "coroutine2(): bool res5 = co_await op5;\n");
    bool res5 = co_await op5;
    print(PRI1, "coroutine2(): res5 = %d\n", res5);
    
    print(PRI1, "coroutine2(): async_remove_oper op6 = start_remove();\n");
    async_remove_oper op6 = start_remove();
    print(PRI1, "coroutine2(): bool res6 = co_await op6;\n");
    bool res6 = co_await op6;
    print(PRI1, "coroutine2(): res6 = %d\n", res6);

    print(PRI1, "coroutine2(): co_return 0;\n");
    co_return 0;
}

EventQueueThrFunctionX evqueuethr;
async_oper_base* async_oper_bases[32];
int start_index = 0;

int main()
{
    set_print_level(0x03);

    print(PRI1, "main(): task t = coroutine1();\n");
    task t = coroutine1();
    print(PRI1, "main(): t.start();\n");
    t.start();
    print(PRI1, "main(): runEventQueue(evqueuethr);\n");
    runEventQueue(evqueuethr);

    print(PRI1, "main(): task t = coroutine2();\n");
    task t2 = coroutine2();
    print(PRI1, "main(): t2.start();\n");
    t2.start();
    print(PRI1, "main(): runEventQueue(evqueuethr);\n");
    runEventQueue(evqueuethr);

    print(PRI1, "main(): return 0;\n");
    return 0;
}
