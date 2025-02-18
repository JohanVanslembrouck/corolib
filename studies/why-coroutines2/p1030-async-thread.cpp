/**
 * @file p1030-async-thread.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <assert.h>
#include <thread>

#include "corolib/print.h"

#include "eventqueuethr.h"
#include "operationsthr.h"

using namespace corolib;

void asynchronous_start_op(int st) {
    int i = st;
    print(PRI1, "asynchronous_start_op: begin\n");

    auto st_create = start_create(i + 1);
    i = st_create.get_result();
    assert(i == st + 1);

    print(PRI1, "asynchronous_start_op: %d: create complete: open\n", i);
    auto st_open = start_open(i + 1);
    i = st_open.get_result();
    assert(i == st + 2);

    print(PRI1, "asynchronous_start_op: %d: open complete: write\n", i);
    auto st_write = start_write(i + 1);
    i = st_write.get_result();
    assert(i == st + 3);

    print(PRI1, "asynchronous_start_op: %d: write complete: read\n", i);
    auto st_read = start_read(i + 1);
    i = st_read.get_result();
    assert(i == st + 4);

    print(PRI1, "asynchronous_start_op: %d: read complete: close\n", i);
    auto st_close = start_close(i + 1);
    i = st_close.get_result();
    assert(i == st + 5);

    print(PRI1, "asynchronous_start_op: %d: close complete: remove\n", i);
    auto st_remove = start_remove(i + 1);
    i = st_remove.get_result();
    assert(i == st + 6);

    print(PRI1, "asynchronous_start_op: %d: remove complete: done\n", i);
    print(PRI1, "asynchronous_start_op: end\n");
}

void asynchronous_operations(int st) {
    int i = st;
    print(PRI1, "asynchronous: begin\n");

    operations ops;

    ops.start_create(i + 1);
    i = ops.get_result();
    assert(i == st + 1);

    print(PRI1, "asynchronous_operations: %d: create complete: open\n", i);
    ops.start_open(i + 1);
    i = ops.get_result();
    assert(i == st + 2);

    print(PRI1, "asynchronous_operations: %d: open complete: write\n", i);
    ops.start_write(i + 1);
    i = ops.get_result();
    assert(i == st + 3);

    print(PRI1, "asynchronous_operations: %d: write complete: read\n", i);
    ops.start_read(i + 1);
    i = ops.get_result();
    assert(i == st + 4);

    print(PRI1, "asynchronous_operations: %d: read complete: close\n", i);
    ops.start_close(i + 1);
    i = ops.get_result();
    assert(i == st + 5);

    print(PRI1, "asynchronous_operations: %d: close complete: remove\n", i);
    ops.start_remove(i + 1);
    i = ops.get_result();
    assert(i == st + 6);

    print(PRI1, "asynchronous_operations: %d: remove complete: done\n", i);
    print(PRI1, "asynchronous_operations: end\n");
}

int main() {
    print(PRI1, "main: begin\n");
    std::thread thr1([] { asynchronous_start_op(10); });
    std::thread thr2([] { asynchronous_operations(20); });
    std::jthread thr3([] { runEventQueue(evqueuethr, 2 * 6); });
    thr3.detach();
    thr1.join();
    thr2.join();
    print(PRI1, "main: end\n");
    return 0;
}
