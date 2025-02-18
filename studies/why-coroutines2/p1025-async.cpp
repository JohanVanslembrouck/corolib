/**
 * @file p1025-async.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <assert.h>

#include "corolib/print.h"

#include "async.h"

using namespace corolib;

void asynchronous_lambdas(int st) {
    print(PRI1, "asynchronous_lambdas: begin\n");
    async_create(
        [st, i = st + 1]() {
            assert(i == st + 1);
            print(PRI1, "create complete: %d: open\n", i);
            async_open(
                [st, i = i + 1]() {
                    assert(i == st + 2);
                    print(PRI1, "open complete: %d: write\n", i);
                    async_write(
                        [st, i = i + 1]() {
                            assert(i == st + 3);
                            print(PRI1, "write complete: %d: read\n", i);
                            async_read(
                                [st, i = i + 1]() {
                                    assert(i == st + 4);
                                    print(PRI1, "read complete: %d: close\n", i);
                                    async_close(
                                        [st, i = i + 1]() {
                                            assert(i == st + 5);
                                            print(PRI1, "close complete: %d: remove\n", i);
                                            async_remove(
                                                [st, i = i + 1]() {
                                                    assert(i == st + 6);
                                                    print(PRI1, "remove complete: %d: done\n", i);
                                                });
                                        });
                                });
                        });
                });
        });
    print(PRI1, "asynchronous_lambdas: end\n");
}

void start_create(int st, int i);
void start_open(int st, int i);
void start_write(int st, int i);
void start_read(int st, int i);
void start_close(int st, int i);
void start_remove(int st, int i);

void asynchronous_lambdas2(int st) {
    print(PRI1, "asynchronous_lambdas: begin\n");
    start_create(st, st+1);
    print(PRI1, "asynchronous_lambdas: end\n");
}

void start_create(int st, int i) {
    async_create(
        [st, i]() {
            assert(i == st + 1);
            print(PRI1, "create complete: %d: open\n", i);
            start_open(st, i+1);
        });
}

void start_open(int st, int i) {
    async_open(
        [st, i]() {
            assert(i == st + 2);
            print(PRI1, "open complete: %d: write\n", i);
            start_write(st, i+1);
        });
}

void start_write(int st, int i) {
    async_write(
        [st, i]() {
            assert(i == st + 3);
            print(PRI1, "write complete: %d: read\n", i);
            start_read(st, i+1);
        });
}

void start_read(int st, int i) {
    async_read(
        [st, i]() {
            assert(i == st + 4);
            print(PRI1, "read complete: %d: close\n", i);
            start_close(st, i+1);
        });
}

void start_close(int st, int i) {
    async_close(
        [st, i]() {
            assert(i == st + 5);
            print(PRI1, "close complete: %d: remove\n", i);
            start_remove(st, i+1);
        });
}

void start_remove(int st, int i) {
    async_remove(
        [st, i]() {
            assert(i == st + 6);
            print(PRI1, "remove complete: %d: done\n", i);
        });
}

int main() {
    print(PRI1, "main: begin\n");
    asynchronous_lambdas(10);
    asynchronous_lambdas(20);
    asynchronous_lambdas2(30);
    asynchronous_lambdas2(40);
    evqueue.run();
    print(PRI1, "main: end\n");
    return 0;
}
