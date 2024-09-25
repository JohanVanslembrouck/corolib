/**
 *  Filename: p1200.cpp
 *  Description:
 *  Basic example with two coroutines f and g.
 * 
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "config.h"
#include "print.h"
#include "auto_reset_event.h"

#define AWAIT_SUSPEND_RETURNS_VOID 1
#define USE_FINAL_AWAITER 0
#include "p1200.h"

auto_reset_event are1;

#if USE_TRANSFORMED_CODE

#include "helpers.h"
#include "p1200-f.h"
#include "p0200-g.h"

#else

task f(int x) {
    std::thread thread1([]() {
        print(PRI1, "f(): thread1: std::this_thread::sleep_for(std::chrono::milliseconds(1000));\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        print(PRI1, "f(): thread1: are.resume();\n");
        are1.resume();
        print(PRI1, "f(): thread1: return;\n");
        });
    thread1.detach();

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
