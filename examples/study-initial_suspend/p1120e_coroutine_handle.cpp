/**
 * @file p1120e_coroutine_handle.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <iostream>

#include "taske_coroutine_handle.h"

task completes_synchronously() {
    co_return;
}

task loop_synchronously(int count) {
    std::cout << "loop_synchronously(" << count << ")" << std::endl;
    for (int i = 0; i < count; ++i) {
        co_await completes_synchronously();
    }
    std::cout << "loop_synchronously(" << count << ") returning" << std::endl;
}

int main() {
    loop_synchronously(100);
    loop_synchronously(1000);
    loop_synchronously(100'000);
    loop_synchronously(1'000'000);
    return 0;
}

