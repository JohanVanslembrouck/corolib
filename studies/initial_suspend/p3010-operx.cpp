/**
 * @file p3010-operx.cpp
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
    print(PRI1, "coroutine1(): auto op1 = start_open();\n");
    auto op1 = start_open();
    print(PRI1, "coroutine1(): int res1 = co_await op1;\n");
    int res1 = co_await op1;
    print(PRI1, "coroutine1(): res1 = %d\n", res1);

    char buffer[100];
    sprintf(buffer, "Input string\n");
    print(PRI1, "coroutine1(): auto op2 = start_write(buffer);\n");
    auto op2 = start_write(buffer);
    print(PRI1, "coroutine1(): int res2 = co_await op2;\n");
    int res2 = co_await op2;
    print(PRI1, "coroutine1(): res2 = %d\n", res2);

    print(PRI1, "coroutine1(): auto op3 = start_read();\n");
    auto op3 = start_read();
    print(PRI1, "coroutine1(): std::string res3 = co_await op3;\n");
    std::string res3 = co_await op3;
    print(PRI1, "coroutine1(): res3 = %s\n", res3.c_str());

    print(PRI1, "coroutine1(): auto op4 = start_close();\n");
    auto op4 = start_close();
    print(PRI1, "coroutine1(): bool res4 = co_await op4;\n");
    bool res4 = co_await op4;
    print(PRI1, "coroutine1(): res4 = %d\n", res4);

    print(PRI1, "coroutine1(): co_return 0;\n");
    co_return 0;
}

EventQueueThrFunctionX evqueuethr;
async_oper_base* async_oper_bases[32];
int start_index = 0;

int main()
{
    set_print_level(0x01);

    print(PRI1, "main(): task t = coroutine1();\n");
    task t = coroutine1();
    print(PRI1, "main(): t.start();\n");
    t.start();
    print(PRI1, "main(): runEventQueue(evqueuethr);\n");
    runEventQueue(evqueuethr);

    print(PRI1, "main(): return 0;\n");
    return 0;
}
