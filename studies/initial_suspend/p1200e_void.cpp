/**
 * @file p1200e_void.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <iostream>

#include "mini_awaiter0.h"
#include "taske_void.h"

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

    loop_synchronously(are, 100);
    for (int i = 0; i < 100; ++i)
        if (i % 2 == 0) {
            tracker1_obj.nr_resumptions++;
            are.resume();
        }

    loop_synchronously(are, 1000);
    for (int i = 0; i < 1000; ++i)
        if (i % 2 == 0) {
            tracker1_obj.nr_resumptions++;
            are.resume();
        }

    loop_synchronously(are, 10'000);
    for (int i = 0; i < 10'000; ++i)
        if (i % 2 == 0) {
            tracker1_obj.nr_resumptions++;
            are.resume();
        }

    loop_synchronously(are, 100'000);
    for (int i = 0; i < 100'000; ++i)
        if (i % 2 == 0) {
            tracker1_obj.nr_resumptions++;
            are.resume();
        }

    loop_synchronously(are, 1'000'000);
    for (int i = 0; i < 1'000'000; ++i)
        if (i % 2 == 0) {
            tracker1_obj.nr_resumptions++;
            are.resume();
        }

    return 0;
}

