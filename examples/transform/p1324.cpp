/**
 *  Filename: p1324.cpp
 *  Description:
 *  Simplified variant of p0300.cpp with only coroutine f.
 * 
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include "config.h"
#include "print.h"
#include "auto_reset_event.h"

#define FINAL_AWAITER_AWAIT_SUSPEND_RETURNS_BOOL 1
#include "p1300.h"

auto_reset_event are1;

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

int main() {
    priority = 0x07;
    print(PRI1, "main(): task ft = f(5);\n");
    task ft = f(5);
    print(PRI1, "main(): int i = ft.get();\n");
    int i = ft.get();
    print(PRI1, "main(): i = %d\n", i);
    print(PRI1, "main(): return 0;\n");
    return 0;
}
