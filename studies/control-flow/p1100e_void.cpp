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
    int res = 0;
    for (int i = 0; i < count; ++i) {
        counter++;
        print(PRI1, "%d: loop_synchronously(%d): completes_synchronously(%d)\n", counter, count, i);
        task cs = completes_synchronously(i);
        print(PRI1, "loop_synchronously(%d): res += co_await cs;\n", count);
        res += co_await cs;
    }
    print(PRI1, "loop_synchronously(%d): co_return %d;\n", count, res);
    co_return res;
}

int main() {
    set_print_level(0x07);

    counter++;
    print(PRI1, "%d: main(): task ls = loop_synchronously(3);\n", counter);
    task ls = loop_synchronously(2);
    print(PRI1, "main(): ls.start();\n");
    ls.start();
    print(PRI1, "main(): int res = ls.get_result();\n");
    int res = ls.get_result();
    print(PRI1, "res = %d\n", res);
    return 0;
}
