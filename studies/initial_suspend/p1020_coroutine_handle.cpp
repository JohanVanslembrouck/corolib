/**
 * @file  p1020e_coroutine_handle.cpp
 * @brief
 *
 * Source: https://godbolt.org/z/9baieF
 *         https://lewissbaker.github.io/2020/05/11/understanding_symmetric_transfer 
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <iostream>

using namespace std;

#include "task_coroutine_handle.h"
#include "sync_wait_task.h"
#include "manual_executor.h"

task foo(manual_executor& ex) {
    std::cout << "inside foo()\n";
    co_await ex.schedule();
    std::cout << "about to return from foo()\n";
    co_return 0;
}

task bar(manual_executor& ex) {
    std::cout << "about to call foo()\n";
    co_await foo(ex);
    std::cout << "done calling foo()\n";
    co_return 0;
}

int main() {
    manual_executor ex;
    ex.sync_wait(bar(ex));
    return 0;
}
