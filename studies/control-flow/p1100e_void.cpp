/**
 * @file p1100e_void.cpp
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#include <iostream>

#include "taske_void_p.h"

task completes_synchronously(int i) {
    print(PRI1, "completes_synchronously(%d): co_return %d;\n", i, i);
    co_return i;
}

task loop_synchronously(int count) {
    print(PRI1, "loop_synchronously(%d)\n", count);
    int v = 0;
    for (int i = 0; i < count; ++i) {
        counter++;
        print(PRI1, "%d: loop_synchronously(%d): task cs = completes_synchronously(%d)\n", counter, count, i);
        task cs = completes_synchronously(i);
        print(PRI1, "loop_synchronously(%d): v += co_await cs;\n", count);
        v += co_await cs;
    }
    print(PRI1, "loop_synchronously(%d): co_return %d;\n", count, v);
    co_return v;
}

int main() {
    set_print_level(0x01);      // Use 0x07 to see the detailed control flow.

    counter++;
    print(PRI1, "%d: main(): task ls = loop_synchronously(4);\n", counter);
    task ls = loop_synchronously(4);
    print(PRI1, "main(): ls.start();\n");
    ls.start();
    print(PRI1, "main(): int v = ls.get_result();\n");
    int v = ls.get_result();
    print(PRI1, "v = %d\n", v);
    return 0;
}
