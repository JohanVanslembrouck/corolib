/**
 * @file p1041-coroutine.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <assert.h>

#include "corolib/print.h"

#include "operations-coroutine.h"
#include "task.h"

using namespace corolib;

task coroutine_start_op(int st) {
    int i = st;
    print(PRI1, "coroutine_start_op: begin\n");

    print(PRI1, "coroutine_start_op: %d: create\n", i);
    i = co_await start_create(i + 1);
    assert(i == st + 1);

    print(PRI1, "coroutine_start_op: %d: create complete: open\n", i);
    i = co_await start_open(i + 1);
    assert(i == st + 2);

    print(PRI1, "coroutine_start_op: %d: open complete: write\n", i);
    i = co_await start_write(i + 1);
    assert(i == st + 3);

    print(PRI1, "coroutine_start_op: %d: write complete: read\n", i);
    i = co_await start_read(i + 1);
    assert(i == st + 4);

    print(PRI1, "coroutine_start_op: %d: read complete: write 2\n", i);       // Second write after read
    i = co_await start_write(i + 1);
    assert(i == st + 5);

    print(PRI1, "coroutine_start_op: %d: write 2 complete: close\n", i);
    i = co_await start_close(i + 1);
    assert(i == st + 6);

    print(PRI1, "coroutine_start_op: %d: close complete: remove\n", i);
    i = co_await start_remove(i + 1);
    assert(i == st + 7);

    print(PRI1, "coroutine_start_op: %d: remove complete: done\n", i);
    print(PRI1, "coroutine_start_op: end\n");
    co_return;
}

task coroutine_operations(int st) {
    int i = st;
    print(PRI1, "coroutine_operations: begin\n");

    operations ops;

    print(PRI1, "coroutine_operations: %d: create\n", st);
    i = co_await ops.start_create(i + 1);
    assert(i == st + 1);

    print(PRI1, "coroutine_operations: %d: create complete: open\n", i);
    i = co_await ops.start_open(i + 1);
    assert(i == st + 2);

    print(PRI1, "coroutine_operations: %d: open complete: write\n", i);
    i = co_await ops.start_write(i + 1);
    assert(i == st + 3);

    print(PRI1, "coroutine_operations: %d: write complete: read\n", i);
    i = co_await ops.start_read(i + 1);
    assert(i == st + 4);

    print(PRI1, "coroutine_operations: %d: read complete: write\n", i);     // Second write after read
    i = co_await ops.start_write(i + 1);
    assert(i == st + 5);

    print(PRI1, "coroutine_operations: %d: read complete: close\n", i);
    i = co_await ops.start_close(i + 1);
    assert(i == st + 6);

    print(PRI1, "coroutine_operations: %d: close complete: remove\n", i);
    i = co_await ops.start_remove(i + 1);
    assert(i == st + 7);

    print(PRI1, "coroutine_operations: %d: remove complete: done\n", i);
    print(PRI1, "coroutine_operations: end\n");
    co_return;
}

int main() {
    print(PRI1, "main: begin\n");
    // Assign to t1/t2/t2 to keep the coroutine object alive until the end of main
    task t1 = coroutine_start_op(10);
    task t2 = coroutine_operations(20);
    task t3 = coroutine_operations(30);
    evqueue.run();
    print(PRI1, "main: end\n");
    return 0;
}
