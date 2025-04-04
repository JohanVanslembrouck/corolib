/**
 * @file p1100_void.cpp
 * @brief
 *
 * Source: https://godbolt.org/z/gy5Q8q 
 *         https://lewissbaker.github.io/2020/05/11/understanding_symmetric_transfer 
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <iostream>

#include "task_void.h"
#include "sync_wait_task.h"
#include "manual_executor.h"

task completes_synchronously() {
    co_return 0;
}

task loop_synchronously(int count) {
    std::cout << "loop_synchronously(" << count << ")" << std::endl;
    for (int i = 0; i < count; ++i) {
        co_await completes_synchronously();
    }
    std::cout << "loop_synchronously(" << count << ") returning" << std::endl;
    co_return 0;
}

int main() {
    manual_executor ex;
    ex.sync_wait(loop_synchronously(100));
#if 0
    ex.sync_wait(loop_synchronously(1000));
    ex.sync_wait(loop_synchronously(10'000));
    ex.sync_wait(loop_synchronously(100'000));
    ex.sync_wait(loop_synchronously(1'000'000));
#endif
    return 0;
}
