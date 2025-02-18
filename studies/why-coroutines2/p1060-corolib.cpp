/**
 * @file p1060-corolib.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <assert.h>

#include "corolib/print.h"
#include "corolib/async_task.h"

#include "operations-corolib.h"

using namespace corolib;

async_task<void> coroutine_corolib(int st) {
    int i = st;
    print(PRI1, "coroutine_corolib: begin\n");

    operationsCL ops;

    print(PRI1, "coroutine_corolib: %d: create\n", i);
    i = co_await ops.start_create(i + 1);
    assert(i == st + 1);

    print(PRI1, "coroutine_corolib: %d: create complete: open\n", i);
    i = co_await ops.start_open(i + 1);
    assert(i == st + 2);

    print(PRI1, "coroutine_corolib: %d: open complete: write\n", i);
    i = co_await ops.start_write(i + 1);
    assert(i == st + 3);

    print(PRI1, "coroutine_corolib: %d: write complete: read\n", i);
    i = co_await ops.start_read(i + 1);
    assert(i == st + 4);

    print(PRI1, "coroutine_corolib: %d: read complete: close\n", i);
    i = co_await ops.start_close(i + 1);
    assert(i == st + 5);

    print(PRI1, "coroutine_corolib: %d: close complete: remove\n", i);
    i = co_await ops.start_remove(i + 1);
    assert(i == st + 6);

    print(PRI1, "coroutine_corolib: %d: remove complete: done\n", i);
    print(PRI1, "coroutine_corolib: end\n");

    co_return;
}

int main() {
    print(PRI1, "main: begin\n");
    // Assign to t1/t2 to keep the coroutine object alive until the end of main
    async_task<void> t1 = coroutine_corolib(10);
    async_task<void> t2 = coroutine_corolib(20);
    evqueue.run();
    print(PRI1, "main: end\n");
    return 0;
}
