/**
 *  Filename: p0320.cpp
 *  Description:
 *  Basic example with two coroutines f and g.
 * 
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "config.h"
#include "print.h"
#include "auto_reset_event.h"

#define FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL 1
#include "p0300.h"

auto_reset_event are1;

task f(int x) {
    print(PRI1, "f(%d): co_await are1;\n", x);
    co_await are1;
    print(PRI1, "f(%d): co_return 42 + x (= %d);\n", x, 42 + x);
    co_return 42 + x;
}

task g(int x) {
    print(PRI1, "g(%d): int i = co_await f(%d);\n", x, x);
    int i = co_await f(x);
    print(PRI1, "g(%d): co_return 42 + i (= %d);\n", x, 42 + i);
    co_return 42 + i;
}

int main() {
    priority = 0x07;
    print(PRI1, "main(): task gt = g(5);\n");
    task gt = g(5);
    print(PRI1, "main(): are1.resume();\n");
    are1.resume();
    print(PRI1, "main(): int i = gt.get();\n");
    int i = gt.get();
    print(PRI1, "main(): i = %d\n", i);
    print(PRI1, "main(): return 0;\n");
    return 0;
}
