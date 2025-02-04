/**
 * @file p2070e_void-op2e-thread.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <iostream>
#include <thread>

#include "operation2e.h"
#include "taske_void_p.h"
#include "print.h"

task foo() {
    print(PRI1, "foo(): operation2e op;\n");
    operation2e op;
    print(PRI1, "foo(): int v = co_await op;\n");
    int v = co_await op;
    print(PRI1, "foo(): co_return %d;\n", v+1);
    co_return v+1;
}

task bar() {
    print(PRI1, "bar(): task f = foo();\n");
    task f = foo();
    print(PRI1, "bar(): int v = co_await f;\n");
    int v = co_await f;
    print(PRI1, "bar(): co_return %d;\n", v+1);
    co_return v+1;
}

EventQueueThrFunctionVoidVoid eventQueueThr;

void completionflow()
{
    print(PRI1, "completionflow(): std::this_thread::sleep_for(std::chrono::milliseconds(0));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(0));

    runEventQueue(eventQueueThr, 1);
}

int main() {
    set_print_level(0x07);
    print(PRI1, "main(): task b = bar();\n");
    task b = bar();
    print(PRI1, "main(): b.start();\n");
    b.start();
    completionflow();

    // This delay gives the completion thread the time to run
    printf("main(): std::this_thread::sleep_for(std::chrono::milliseconds(1100));\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));

    print(PRI1, "main(): int v = b.get_result();\n");
    int v = b.get_result();
    print(PRI1, "main(): v = %d;\n", v);
    print(PRI1, "main(): return 0;\n");
    return 0;
}
