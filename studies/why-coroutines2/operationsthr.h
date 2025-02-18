/**
 * @file operationsthr.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#pragma once

#include "corolib/print.h"

#include "thread_awaiter.h"
#include "asyncthr.h"

using namespace corolib;

struct start_create : public thread_awaiter {
    start_create(int i) {
        print(PRI1); print(PRI1, "start_create(%d)\n", i);
        async_create([this, i]() {
            print(PRI1, "start_create(): begin\n");
            set_result_and_release(i);
            print(PRI1, "start_create(): end\n");
            });
    }
};

struct start_open : public thread_awaiter {
    start_open(int i) {
        print(PRI1); print(PRI1, "start_open(%d)\n", i);
        async_open([this, i]() {
            print(PRI1, "start_open(): begin\n");
            set_result_and_release(i);
            print(PRI1, "start_open(): end\n");
            });
    }
};

struct start_write : public thread_awaiter {
    start_write(int i) {
        print(PRI1); print(PRI1, "start_write(%d)\n", i);
        async_write([this, i]() {
            print(PRI1, "start_write(): begin\n");
            set_result_and_release(i);
            print(PRI1, "start_write(): end\n");
            });
    }
};

struct start_read : public thread_awaiter {
    start_read(int i) {
        print(PRI1); print(PRI1, "start_read(%d)\n", i);
        async_read([this, i]() {
            print(PRI1, "start_read(): begin\n");
            set_result_and_release(i);
            print(PRI1, "start_read(): end\n");
            });
    }
};

struct start_close : public thread_awaiter {
    start_close(int i) {
        print(PRI1); print(PRI1, "start_close(%d)\n", i);
        async_close([this, i]() {
            print(PRI1, "start_close(): begin\n");
            set_result_and_release(i);
            print(PRI1, "start_close(): end\n");
            });
    }
};

struct start_remove : public thread_awaiter {
    start_remove(int i) {
        print(PRI1); print(PRI1, "start_remove(%d)\n", i);
        async_remove([this, i]() {
            print(PRI1, "start_remove(): begin\n");
            set_result_and_release(i);
            print(PRI1, "start_remove(): end\n");
            });
    }
};

struct operations : public thread_awaiter {
    void start_create(int i) {
        print(PRI1); print(PRI1, "start_create(%d)\n", i);
        async_create([this, i]() {
            print(PRI1, "async_create handler: begin\n");
            set_result_and_release(i);
            print(PRI1, "async_create handler: end\n");
            });
    }

    void start_open(int i) {
        print(PRI1); print(PRI1, "start_open(%d)\n", i);
        async_open([this, i]() {
            print(PRI1, "async_open handler: begin\n");
            set_result_and_release(i);
            print(PRI1, "async_open handler: end\n");
            });
    }

    void start_write(int i) {
        print(PRI1); print(PRI1, "start_write(%d)\n", i);
        async_write([this, i]() {
            print(PRI1, "async_write handler: begin\n");
            set_result_and_release(i);
            print(PRI1, "async_write handler: end\n");
            });
    }

    void start_read(int i) {
        print(PRI1); print(PRI1, "start_read(%d)\n", i);
        async_read([this, i]() {
            print(PRI1, "async_read handler: begin\n");
            set_result_and_release(i);
            print(PRI1, "async_read handler: end\n");
            });
    }

    void start_close(int i) {
        print(PRI1); print(PRI1, "start_close(%d)\n", i);
        async_close([this, i]() {
            print(PRI1, "async_close handler: begin\n");
            set_result_and_release(i);
            print(PRI1, "async_close handler: end\n");
            });
    }

    void start_remove(int i) {
        print(PRI1); print(PRI1, "start_remove(%d)\n", i);
        async_remove([this, i]() {
            print(PRI1, "async_remove handler: begin\n");
            set_result_and_release(i);
            print(PRI1, "async_remove handler: end\n");
            });
    }
};
