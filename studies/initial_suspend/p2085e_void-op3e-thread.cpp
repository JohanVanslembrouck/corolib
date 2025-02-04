/**
 * @file p2085e_void-op3e-thread.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <iostream>
#include <thread>

#include "operation3e.h"
#include "taske_void_p.h"
#include "print.h"

ThreadAwaker awaker;

task foo() {
    print(PRI1, "foo(): operation3e op;\n");
    operation3e op(&awaker, true);
    print(PRI1, "foo(): int v = co_await op;\n");
    int v = co_await op;
    print(PRI1, "foo(): co_return v+1;\n", v+1);
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

void completionflow(ThreadAwaker* awaker)
{
    if (awaker)
        awaker->releaseThreads();
}

int main() {
    set_print_level(0x07);
    print(PRI1, "main(): task b = bar();\n");
    task b = bar();
    print(PRI1, "main(): b.start();\n");
    b.start();
    completionflow(&awaker);

    print(PRI1, "main(): int v = b.get_result();\n");
    int v = b.get_result();
    print(PRI1, "main(): v = %d;\n", v);
    print(PRI1, "main(): return 0;\n");
    return 0;
}
