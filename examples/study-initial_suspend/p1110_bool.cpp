/**
 * @file p1110_bool.cpp
 * @brief
 *
 * Source: https://godbolt.org/z/7fm8Za
 *         https://lewissbaker.github.io/2020/05/11/understanding_symmetric_transfer 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <iostream>

#include "task_bool.h"
#include "sync_wait_task.h"
#include "manual_executor.h"

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
    manual_executor ex;
    ex.sync_wait(loop_synchronously(100));
    ex.sync_wait(loop_synchronously(1000));
    ex.sync_wait(loop_synchronously(100'000));
    ex.sync_wait(loop_synchronously(1'000'000));
    return 0;
}
