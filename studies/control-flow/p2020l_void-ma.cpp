/**
 * @file p2020l_void-ma.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <iostream>

#include "mini_awaiter_p.h"
#include "taskl_void_p.h"
#include "print.h"
#include "counter.h"

mini_awaiter ma;

task foo() {
    print(PRI1, "foo(): int v = co_await ma;\n");
    int v = co_await ma;
    print(PRI1, "foo(): co_return %d;\n", v+1);
    co_return v+1;
}

task bar() {
    counter++;
    print(PRI1, "%d: bar(): task f = foo();\n", counter);
    task f = foo();
    print(PRI1, "bar(): int v = co_await f;\n");
    int v = co_await f;
    print(PRI1, "bar(): co_return %d;\n", v+1);
    co_return v+1;
}

int main() {
    set_print_level(0x07);

    counter++;
    print(PRI1, "%d: main(): task b = bar();\n", counter);
    task b = bar();
    print(PRI1, "main(): b.start();\n");
    b.start();
    print(PRI1, "main(): ma.set_result_and_resume(10);\n");
    ma.set_result_and_resume(10);

    print(PRI1, "main(): int v = b.get_result();\n");
    int v = b.get_result();
    print(PRI1, "main(): v = %d;\n", v);
    print(PRI1, "main(): return 0;\n");
    return 0;
}
