/**
 * @file p1000_void.cpp
 * @brief
 *
 * Source: https://godbolt.org/z/-Kw6Nf
 *         https://lewissbaker.github.io/2020/05/11/understanding_symmetric_transfer 
 * 
 * @author Lewis Baker
 */

#include <iostream>

#include "task_void.h"
#include "sync_wait_task.h"
#include "manual_executor.h"

task foo(manual_executor& ex) {
    std::cout << "inside foo()\n";
    co_await ex.schedule();
    std::cout << "about to return from foo()\n";
    co_return;
}

task bar(manual_executor& ex) {
    std::cout << "about to call foo()\n";
    co_await foo(ex);
    std::cout << "done calling foo()\n";
}

int main() {
    manual_executor ex;
    ex.sync_wait(bar(ex));
    return 0;
}

