/**
 * @file p2020_void.cpp
 * @brief
 *
 * Source: https://godbolt.org/z/-Kw6Nf
 *         https://lewissbaker.github.io/2020/05/11/understanding_symmetric_transfer 
 * 
 * @author Lewis Baker
 */

#include <iostream>

#include "task_void_p.h"
#include "sync_wait_task.h"
#include "manual_executor.h"
#include "print.h"

manual_executor ex;

task foo() {
    print(PRI1, "foo(): co_await ex.schedule();\n");
    co_await ex.schedule();
    print(PRI1, "foo(): co_return 1;\n");
    co_return 1;
}

task bar() {
    print(PRI1, "bar(): int v = co_await foo();\n");
    int v = co_await foo();
    print(PRI1, "bar(): co_return %d;\n", v+1);
    co_return v+1;
}

int main() {
    print(PRI1, "main(): task b = bar();\n");
    task b = bar();
    print(PRI1, "main(): ex.sync_wait(std::move(b))\n");
    ex.sync_wait(std::move(b));

    print(PRI1, "main(): int v = b.get_result();\n");
    int v = b.get_result();
    print(PRI1, "main(): v = %d;\n", v);
    print(PRI1, "main(): return 0;\n");
    return 0;
}
