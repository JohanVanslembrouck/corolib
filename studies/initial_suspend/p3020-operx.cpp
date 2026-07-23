/**
 * @file p3020-operx.cpp
 * @brief Contains the main code that is included in
 * 
 * p3020e_void-opere.cpp
 * p3020e_void-operl.cpp
 * p3020l_void-opere.cpp
 * p3020l_void-operl.cpp
 *
 * and that will be compiled in 4 different ways by including different header files.
 * 
 * This file is a variant of p3010-operx.cpp that was introduced to study reversing the order
 * of the async_open, async_write, async_read and async_close functions.
 * 
 * @author Johan Vanslembrouck
 */

task coroutine_open()
{
    print(PRI2, "coroutine_open(): auto op = start_open();\n");
    auto op = start_open();
    print(PRI2, "coroutine_open(): int res = co_await op;\n");
    int res = co_await op;
    print(PRI2, "coroutine_open(): res = %d\n", res);
    print(PRI1, "open completion handled\n");

    print(PRI2, "coroutine_open(): co_return 0;\n");
    co_return 0;
}

/**
 * @brief
 * In case of eager start coroutines and operations,
 * async_write is called before async_open may have completed.
 */
task coroutine_write_after_open()
{
    print(PRI2, "coroutine_write_after_open(): task t = coroutine_open();\n");
    task t = coroutine_open();
    char buffer[100];
    sprintf(buffer, "Input string\n");
    print(PRI2, "coroutine_write_after_open(): auto op = start_write(buffer);\n");
    auto op = start_write(buffer);

    print(PRI2, "coroutine_write_after_open(): int resT = co_await t;\n");
    int rest = co_await t;
    print(PRI2, "coroutine_write_after_open(): rest = %d\n", rest);

    print(PRI2, "coroutine_write_after_open(): int res = co_await op;\n");
    int res = co_await op;
    print(PRI2, "coroutine_write_after_open(): res = %d\n", res);
    print(PRI1, "write completion handled\n");

    print(PRI2, "coroutine_write_after_open(): co_return 0;\n");
    co_return 0;
}

/**
 * @brief
 * In case of eager start coroutines and operations,
 * async_read is called before async_write may have completed.
 */
task coroutine_read_after_write()
{
    print(PRI2, "coroutine_read_after_write(): task t = coroutine_write_after_open();\n");
    task t = coroutine_write_after_open();
    print(PRI2, "coroutine_read_after_write(): auto op = start_read();\n");
    auto op = start_read();

    print(PRI2, "coroutine_read_after_write(): int rest = co_await t;\n");
    int rest = co_await t;
    print(PRI2, "coroutine_read_after_write(): rest = %d;\n", rest);

    print(PRI2, "coroutine_read_after_write(): res = co_await op;\n");
    std::string res = co_await op;
    print(PRI2, "coroutine_read_after_write(): res2 = %s\n", res.c_str());
    print(PRI1, "read completion handled\n");

    print(PRI2, "coroutine_read_after_write(): co_return 0;\n");
    co_return 0;
}

/**
 * @brief
 * In case of eager start coroutines and operations,
 * async_close is called before async_read may have completed.
 */
task coroutine_close_after_read()
{
    print(PRI2, "coroutine_close_after_read(): task t = coroutine_read_after_write();\n");
    task t = coroutine_read_after_write();
    print(PRI2, "coroutine_close_after_read(): auto op = start_close();\n");
    auto op = start_close();

    print(PRI2, "coroutine_close_after_read(): int rest = co_await t;\n");
    int rest = co_await t;
    print(PRI2, "coroutine_close_after_read(): rest = %d;\n", rest);

    print(PRI2, "coroutine_close_after_read(): bool res = co_await op;\n");
    bool res = co_await op;
    print(PRI2, "coroutine_close_after_read(): res = %d\n", res);
    print(PRI1, "close completion handled\n");

    print(PRI2, "coroutine_close_after_read(): co_return 0;\n");
    co_return 0;
}

EventQueueThrFunctionX evqueuethr;
async_oper_base* async_oper_bases[32];
int start_index = 0;

int main()
{
    set_print_level(0x01);

    print(PRI2, "main(): task t = coroutine_close_after_read();\n");
    task t = coroutine_close_after_read();
    print(PRI2, "main(): t.start();\n");
    t.start();
    print(PRI2, "main(): runEventQueue(evqueuethr);\n");
    runEventQueue(evqueuethr);

    print(PRI2, "main(): return 0;\n");
    return 0;
}
