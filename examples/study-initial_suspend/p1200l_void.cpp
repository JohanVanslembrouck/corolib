/**
 * @file p1200l_void.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <iostream>

#include "mini_awaiter.h"
#include "task_void.h"

task completes_synchronously(mini_awaiter& are, int i) {
    if (i % 2 == 0)
        co_await are;
    else
        co_return;
}

task loop_synchronously(mini_awaiter& are, int count) {
    std::cout << "loop_synchronously(" << count << ")" << std::endl;
    for (int i = 0; i < count; ++i) {
        co_await completes_synchronously(are, i);
    }
    std::cout << "loop_synchronously(" << count << ") returning" << std::endl;
}

int main() {
    mini_awaiter are;

    loop_synchronously(are, 100).start();
    for (int i = 0; i < 100; ++i)
        if (i % 2 == 0)
            are.resume();

    loop_synchronously(are, 1000).start();
    for (int i = 0; i < 1000; ++i)
        if (i % 2 == 0)
            are.resume();

    loop_synchronously(are, 100'000).start();
    for (int i = 0; i < 100'000; ++i)
        if (i % 2 == 0)
            are.resume();

    loop_synchronously(are, 1'000'000).start();
    for (int i = 0; i < 1'000'000; ++i)
        if (i % 2 == 0)
            are.resume();

    return 0;
}
