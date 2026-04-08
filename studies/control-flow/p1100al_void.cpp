/**
 * @file p1100al_void.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <iostream>

#include "taskl_void_p.h"

task completes_synchronously(int i) {
    co_return i;
}

task loop_synchronously(int count) {
    int v = 0;
    for (int i = 0; i < count; ++i) {
        task cs = completes_synchronously(i);
        v += co_await cs;
    }
    co_return v;
}

int main() {
    set_print_level(0x01);

    print(PRI1, "main(): task ls = loop_synchronously(10000);\n");
    task ls = loop_synchronously(10000);
    print(PRI1, "main(): ls.start();\n");
    ls.start();
    print(PRI1, "main(): int v = ls.get_result();\n");
    int v = ls.get_result();
    print(PRI1, "v = %d\n", v);
    return 0;
}
