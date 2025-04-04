/**
 * @file p1020e_coroutine_handle.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <iostream>

#include <coroutine>
#include <utility>

using namespace std;

#include "mini_awaiter0.h"
#include "taske_coroutine_handle.h"

task foo(mini_awaiter& are) {
    std::cout << "inside foo()\n";
    co_await are;
    std::cout << "about to return from foo()\n";
    co_return 0;
}

task bar(mini_awaiter& ex) {
    std::cout << "about to call foo()\n";
    co_await foo(ex);
    std::cout << "done calling foo()\n";
    co_return 0;
}

int main() {
    mini_awaiter are;
    bar(are);
    tracker1_obj.nr_resumptions++;
    are.resume();
    return 0;
}
