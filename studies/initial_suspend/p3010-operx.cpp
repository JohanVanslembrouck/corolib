/**
 * @file p3000-operx.cpp
 * @brief Contains the main code that is included in
 * 
 * p3010e_void-opere.cpp
 * p3010e_void-operl.cpp
 * p3010l_void-opere.cpp
 * p3010l_void-operl.cpp
 *
 * and that will be compiled in 4 different ways by including different header files.
 * 
 * @author Johan Vanslembrouck
 */

task coroutine1()
{
    char buffer[100];
    print(PRI1, "coroutine1: auto op = start_write(buffer);\n");
    auto op = start_write(buffer);
    print(PRI1, "coroutine1: int res = co_await op;\n");
    int res = co_await op;
    print(PRI1, "coroutine1: res = %d;\n", res);
    print(PRI1, "coroutine1: co_return 0;\n");
    co_return 0;
}

task coroutine2()
{
    int res = 0;

    print(PRI1, "coroutine2: auto op = start_create();\n");
    auto op1 = start_create();
    print(PRI1, "coroutine1: res = co_await op1;\n");
    res = co_await op1;
    print(PRI1, "coroutine2: res = %d;\n", res);

    print(PRI1, "coroutine2: auto op2 = start_open();\n");
    auto op2 = start_open();
    print(PRI1, "coroutine1: res = co_await op2;\n");
    res = co_await op2;
    print(PRI1, "coroutine2: res = %d;\n", res);

    char buffer[100];
    print(PRI1, "coroutine2: auto op3 = start_write(buffer);\n");
    auto op3 = start_write(buffer);
    print(PRI1, "coroutine2: int res = co_await op3;\n");
    res = co_await op3;
    print(PRI1, "coroutine2: res = %d;\n", res);

    print(PRI1, "coroutine2: auto op4 = start_read();\n");
    auto op4 = start_read();
    print(PRI1, "coroutine1: res = co_await op4;\n");
    res = co_await op4;
    print(PRI1, "coroutine2: res = %d;\n", res);

    print(PRI1, "coroutine2: auto op5 = start_close();\n");
    auto op5 = start_close();
    print(PRI1, "coroutine1: res = co_await op5;\n");
    res = co_await op5;
    print(PRI1, "coroutine2: res = %d;\n", res);

    print(PRI1, "coroutine2: auto op6 = start_close();\n");
    auto op6 = start_remove();
    print(PRI1, "coroutine1: res = co_await op6;\n");
    res = co_await op6;
    print(PRI1, "coroutine2: res = %d;\n", res);

    print(PRI1, "coroutine2: co_return;\n");
    co_return 0;
}

EventQueueThrFunctionVoidVoid evqueuethr;
async_oper_base* async_oper_bases[32];
int start_index = 0;

int main()
{
    set_print_level(0x03);

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
