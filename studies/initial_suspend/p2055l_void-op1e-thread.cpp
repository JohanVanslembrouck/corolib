/**
 * @file p2055e_void-op1e-thread.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <iostream>
#include <thread>

#include "operation1e.h"
#include "task_void_p.h"
#include "print.h"

task foo() {
    print(PRI1, "foo(): operation1e op;\n");
    operation1e op(true);
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

int main() {
    set_print_level(0x07);
    print(PRI1, "main(): task b = bar();\n");
    task b = bar();
    print(PRI1, "main(): b.start();\n");
    b.start();

    print(PRI1, "main(): int v = b.get_result();\n");
    int v = b.get_result();
    print(PRI1, "main(): v = %d;\n", v);
    print(PRI1, "main(): return 0;\n");
    return 0;
}
