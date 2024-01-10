/**
 *  Filename: p0256.cpp
 *  Description:
 *  Same as p0216.cpp apart from the use of AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE = 1.
 * 
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "config.h"
#include "print.h"
#include "auto_reset_event.h"

#define AWAIT_SUSPEND_RETURNS_COROUTINE_HANDLE 1
#define USE_FINAL_AWAITER 1
#include "p0200.h"

auto_reset_event are1;

#if USE_TRANSFORMED_CODE

#include "helpers.h"
#include "p0200-f.h"
#include "p0240-g.h"
#include "p0240-h.h"

#else

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

task h(int x, int y) {
    print(PRI1, "h(%d, %d): int i = co_await g(%d);\n", x, y, x);
    int i = co_await g(x);
    print(PRI1, "h(%d, %d): co_return y + i (= %d);\n", x, y, y + i);
    co_return y + i;
}
#endif

int main() {
    priority = 0x07;
    print(PRI1, "main(): task ht = h(5, 10);\n");
    task ht = h(5, 10);
    //print(PRI1, "main(): are1.resume();\n");
    //are1.resume();
    print(PRI1, "main(): int i = ht.get();\n");
    int i = ht.get();
    print(PRI1, "main(): i = %d\n", i);
    print(PRI1, "main(): return 0;\n");
    return 0;
}
