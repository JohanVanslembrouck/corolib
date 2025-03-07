/**
 * @file p1072-corolib-thread.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <assert.h>
#include <thread>

#include "corolib/async_task.h"
#include "corolib/print.h"

#include "operations-corolibthr.h"

using namespace corolib;

async_task<void> coroutine_corolib(int st) {
    int i = st;
    print(PRI1, "coroutine_corolib: begin\n");

    operationsCL ops;

    print(PRI1, "coroutine_corolib: %d: create\n", i);
    auto op1 = ops.start_create(i + 1);
    i = co_await op1;
    assert(i == st + 1);

    print(PRI1, "coroutine_corolib: %d: create complete: open\n", i);
    auto op2 = ops.start_open(i + 1);
    i = co_await op2;
    assert(i == st + 2);

    print(PRI1, "coroutine_corolib: %d: open complete: write\n", i);
    auto op3 = ops.start_write(i + 1);
    i = co_await op3;
    assert(i == st + 3);

    print(PRI1, "coroutine_corolib: %d: write complete: read\n", i);
    auto op4 = ops.start_read(i + 1);
    i = co_await op4;
    assert(i == st + 4);

    print(PRI1, "coroutine_corolib: %d: read complete: write 2\n", i);      // Second write after read
    auto op5 = ops.start_write(i + 1);
    i = co_await op5;
    assert(i == st + 5);

    print(PRI1, "coroutine_corolib: %d: write 2 complete: close\n", i);
    auto op6 = ops.start_close(i + 1);
    i = co_await op6;
    assert(i == st + 6);

    print(PRI1, "coroutine_corolib: %d: close complete: remove\n", i);
    auto op7 = ops.start_remove(i + 1);
    i = co_await op7;
    assert(i == st + 7);

    print(PRI1, "coroutine_corolib: %d: remove complete: done\n", i);
    print(PRI1, "coroutine_corolib: end\n");

    co_return;
}

int main() {
    print(PRI1, "main: begin\n");
    // Assign to t1/t2 to keep the coroutine object alive until the end of main
    async_task<void> t1 = coroutine_corolib(10);
    async_task<void> t2 = coroutine_corolib(20);
    std::jthread thr([] { runEventQueue(evqueuethr, 2 * 7); });
    //thr.detach();
    t1.wait();
    t2.wait();
    print(PRI1, "main: end\n");
    return 0;
}
