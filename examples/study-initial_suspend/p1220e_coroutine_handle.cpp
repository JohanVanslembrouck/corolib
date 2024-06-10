/**
 * @file p1220e_coroutine_handle.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <iostream>

#include "mini_awaiter.h"
#include "taske_coroutine_handle.h"

task completes_synchronously(mini_awaiter& are) {
    static int cntr = 0;
    if (cntr++ % 2 == 0)
        co_await are;
    else
        co_return;
}

task loop_synchronously(mini_awaiter& are, int count) {
    std::cout << "loop_synchronously(" << count << ")" << std::endl;
    for (int i = 0; i < count; ++i) {
        co_await completes_synchronously(are);
    }
    std::cout << "loop_synchronously(" << count << ") returning" << std::endl;
}

int main() {
    mini_awaiter are;

    loop_synchronously(are, 100);
    for (int i = 0; i < 100; ++i)
        if (i % 2 == 0)
            are.resume();

    loop_synchronously(are, 1000);
    for (int i = 0; i < 1000; ++i)
        if (i % 2 == 0)
            are.resume();

    loop_synchronously(are, 100'000);
    for (int i = 0; i < 100'000; ++i)
        if (i % 2 == 0)
            are.resume();

    loop_synchronously(are, 1'000'000);
    for (int i = 0; i < 1'000'000; ++i)
        if (i % 2 == 0)
            are.resume();

    return 0;
}

