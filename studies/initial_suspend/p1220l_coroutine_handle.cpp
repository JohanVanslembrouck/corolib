/**
 * @file p1220l_coroutine_handle.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <iostream>

#include "mini_awaiter0.h"
#include "task_coroutine_handle.h"

task completes_synchronously(mini_awaiter& are) {
    static int cntr = 0;
    if (cntr++ % 2 == 0)
        co_await are;
    else
        co_return 0;
}

task loop_synchronously(mini_awaiter& are, int count) {
    std::cout << "loop_synchronously(" << count << ")" << std::endl;
    for (int i = 0; i < count; ++i) {
        co_await completes_synchronously(are);
    }
    std::cout << "loop_synchronously(" << count << ") returning" << std::endl;
    co_return 0;
}

int main() {
    mini_awaiter are;

    loop_synchronously(are, 100).start();
    for (int i = 0; i < 100; ++i)
        if (i % 2 == 0) {
            tracker1_obj.nr_resumptions++;
            are.resume();
        }

    loop_synchronously(are, 1000).start();
    for (int i = 0; i < 1000; ++i)
        if (i % 2 == 0) {
            tracker1_obj.nr_resumptions++;
            are.resume();
        }

    loop_synchronously(are, 10'000).start();
    for (int i = 0; i < 10'000; ++i)
        if (i % 2 == 0) {
            tracker1_obj.nr_resumptions++;
            are.resume();
        }

    loop_synchronously(are, 100'000).start();
    for (int i = 0; i < 100'000; ++i)
        if (i % 2 == 0) {
            tracker1_obj.nr_resumptions++;
            are.resume();
        }

    loop_synchronously(are, 1'000'000).start();
    for (int i = 0; i < 1'000'000; ++i)
        if (i % 2 == 0) {
            tracker1_obj.nr_resumptions++;
            are.resume();
        }

    return 0;
}

