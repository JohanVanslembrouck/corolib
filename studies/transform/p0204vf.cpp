/**
 *  Filename: p0204vf.cpp
 *  Description:
 *  Simplified variant of p0200.cpp with only coroutine f.
 * 
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include "configvf.h"
#include "print.h"
#include "auto_reset_event.h"

#define AWAIT_SUSPEND_RETURNS_VOID 1
#define USE_FINAL_AWAITER 0
#include "p0200.h"

auto_reset_event are1;

#if USE_TRANSFORMED_CODE

#include "helpers.h"
#include "p0200vf-f.h"

#else

task f(int x) {
    print(PRI1, "f(%d): co_await are1;\n", x);
    co_await are1;
    print(PRI1, "f(%d): co_return 42 + x (= %d);\n", x, 42 + x);
    co_return 42 + x;
}
#endif

int main() {
    priority = 0x07;
    print(PRI1, "main(): task ft = f(5);\n");
    task ft = f(5);
    print(PRI1, "main(): are1.resume();\n");
    are1.resume();
    print(PRI1, "main(): int i = ft.get();\n");
    int i = ft.get();
    print(PRI1, "main(): i = %d\n", i);
    print(PRI1, "main(): return 0;\n");
    return 0;
}
