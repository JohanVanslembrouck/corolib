/**
 * @file p1230e_corolib.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <iostream>

#include "mini_awaiter.h"

#include "corolib/async_task.h"
using namespace corolib;

async_task<void> completes_synchronously(mini_awaiter& are, int i) {
    if (i % 2 == 0)
        co_await are;
    else
        co_return;
}

async_task<void> loop_synchronously(mini_awaiter& are, int count) {
    std::cout << "loop_synchronously(" << count << ")" << std::endl;
    for (int i = 0; i < count; ++i) {
        co_await completes_synchronously(are, i);
    }
    std::cout << "loop_synchronously(" << count << ") returning" << std::endl;
}

int main() {
    mini_awaiter are;

    async_task<void> t1 = loop_synchronously(are, 100);
    for (int i = 0; i < 100; ++i)
        if (i % 2 == 0)
            are.resume();
    t1.wait();

    async_task<void> t2 = loop_synchronously(are, 1000);
    for (int i = 0; i < 1000; ++i)
        if (i % 2 == 0)
            are.resume();
    t2.wait();

    async_task<void> t3 = loop_synchronously(are, 10'000);
    t3.start();
    for (int i = 0; i < 10'000; ++i)
        if (i % 2 == 0)
            are.resume();
    t3.wait();

    async_task<void> t4 = loop_synchronously(are, 100'000);
    t4.start();
    for (int i = 0; i < 100'000; ++i)
        if (i % 2 == 0)
            are.resume();
    t4.wait();

    async_task<void> t5 = loop_synchronously(are, 1'000'000);
    t5.start();
    for (int i = 0; i < 1'000'000; ++i)
        if (i % 2 == 0)
            are.resume();
    t5.wait();

    return 0;
}

