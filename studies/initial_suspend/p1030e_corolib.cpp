/**
 * @file p1030e_corolib.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <iostream>

#include <corolib/async_task.h>
#include <corolib/auto_reset_event.h>

using namespace corolib;

async_task<void> foo(auto_reset_event& are) {
    std::cout << "inside foo()\n";
    co_await are;
    std::cout << "about to return from foo()\n";
    co_return;
}

async_task<void> bar(auto_reset_event& ex) {
    std::cout << "about to call foo()\n";
    co_await foo(ex);
    std::cout << "done calling foo()\n";
}

int main() {
    auto_reset_event are;
    async_task<void> tb = bar(are);
    are.resume();
    tb.wait();
    return 0;
}
