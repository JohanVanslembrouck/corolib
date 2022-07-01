/**
 *  Filename: p1406-async_operation-immediate.cpp
 *  Description:
 * 
 *  Tested with Visual Studio 2019.
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 *
 */

#include <functional>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

using namespace corolib;

std::function<void(int)> operation;        // Will be initialized in start_op

void start_op(async_operation<int>* op)
{
    print(PRI1, "start_op()\n");

    operation = [op](int i)
    {
        print(PRI1, "start_op()\n");

        if (op)
        {
            print(PRI1, "start_op(): op->set_result(%d)\n", i);
            op->set_result(i);
            op->completed();
        }
        else
        {
            // This can occur when the async_operation_base has gone out of scope.
            print(PRI1, "start_op() : Warning: op == nullptr\n");
        }
    };

    operation(10);
}

async_task<int> coroutine1();

int main()
{
    set_print_level(0x01);        // Use 0x03 to follow the flow in corolib

    print(PRI1, "main(): async_task<int> a = coroutine1();\n");
    async_task<int> a = coroutine1();

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): int v = a.get_result();\n");
    int v = a.get_result();
    print(PRI1, "main(): v = %d\n", v);

    print(PRI1, "main(): std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    print(PRI1, "main(): return 0;\n");
    return 0;
}
