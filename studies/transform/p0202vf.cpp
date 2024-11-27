/**
 *  Filename: p0202vf.cpp
 *  Description:
 *  Simplified variant of p0200.cpp without use of auto_reset_event.
 *  The coroutines behave like functions, i.e. there is no suspend/resume.
 * 
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include "configvf.h"
#include "print.h"

#define AWAIT_SUSPEND_RETURNS_VOID 1
#define USE_FINAL_AWAITER 0
#include "p0200.h"

#if USE_TRANSFORMED_CODE

#include "helpers.h"
#include "p0202vf-f.h"
#include "p0200vf-g.h"

#else

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
