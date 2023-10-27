/**
 *  Filename: p0204trf.cpp
 *  Description
 * 
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "config.h"

#include "print.h"

#include "auto_reset_event.h"
#include "p0200.h"
#include "helpers.h"

auto_reset_event are1;

#include "p0200-f.h"

#if 0
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
