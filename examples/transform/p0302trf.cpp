/**
 *  Filename: p0302trf.cpp
 *  Description:
 *  Manually transformed version of p0302.cpp.
 * 
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "config.h"
#include "print.h"

#define FINAL_AWAITER_RETURNS_VOID 1
#include "p0300.h"
#include "helpers.h"

#include "p0302-F.h"
#include "p0300-g.h"

#if 0
task f(int x) {
    print(PRI1, "f(%d): co_return 42 + x (= %d);\n", x, 42 + x);
    co_return 42 + x;
}

task g(int x) {
    print(PRI1, "g(%d): int i = co_await f(%d);\n", x, x);
    int i = co_await f(x);
    print(PRI1, "g(%d): co_return 42 + i (= %d);\n", x, 42 + i);
    co_return 42 + i;
}
#endif

int main() {
    priority = 0x07;
    print(PRI1, "main(): task gt = g(5);\n");
    task gt = g(5);
    print(PRI1, "main(): int i = gt.get();\n");
    int i = gt.get();
    print(PRI1, "main(): i = %d\n", i);
    print(PRI1, "main(): return 0;\n");
    return 0;
}
