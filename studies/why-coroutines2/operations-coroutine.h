/**
 * @file operations-coroutine.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#pragma once

#include "corolib/print.h"

#include "mini_awaiter.h"
#include "async.h"

using namespace std;

struct start_create : public mini_awaiter {
    start_create(int i) {
        print(PRI1); print(PRI1, "start_create(%d)\n", i);
        async_create([this, i]() {
            print(PRI1, "start_create handler: begin\n");
            set_result_and_resume(i);
            print(PRI1, "start_create handler: end\n");
            });
    }
};

struct start_open : public mini_awaiter {
    start_open(int i) {
        print(PRI1); print(PRI1, "start_open(%d)\n", i);
        async_open([this, i]() {
            print(PRI1, "async_open handler: begin\n");
            set_result_and_resume(i);
            print(PRI1, "async_open handler: end\n");
            });
    }
};

struct start_write : public mini_awaiter {
    start_write(int i) {
        print(PRI1); print(PRI1, "start_write(%d)\n", i);
        async_write([this, i]() {
            print(PRI1, "async_write handler: begin\n");
            set_result_and_resume(i);
            print(PRI1, "async_write handler: end\n");
            });
    }
};

struct start_read : public mini_awaiter {
    start_read(int i) {
        print(PRI1); print(PRI1, "start_read(%d)\n", i);
        async_read([this, i]() {
            print(PRI1, "async_read handler: begin\n");
            set_result_and_resume(i);
            print(PRI1, "async_read handler: end\n");
            });
    }
};

struct start_close : public mini_awaiter {
    start_close(int i) {
        print(PRI1); print(PRI1, "start_close(%d)\n", i);
        async_close([this, i]() {
            print(PRI1, "async_close handler: begin\n");
            set_result_and_resume(i);
            print(PRI1, "async_close handler: end\n");
            });
    }
};

struct start_remove : public mini_awaiter {
    start_remove(int i) {
        print(PRI1); print(PRI1, "start_remove(%d)\n", i);
        async_remove([this, i]() {
            print(PRI1, "async_remove handler: begin\n");
            set_result_and_resume(i);
            print(PRI1, "async_remove handler: end\n");
            });
    }
};


struct operations : public mini_awaiter {
    operations& start_create(int i) {
        m_ready = false;
        print(PRI1); print(PRI1, "start_create(%d)\n", i);
        async_create([this, i]() {
            print(PRI1, "async_create handler: begin\n");
            set_result_and_resume(i);
            print(PRI1, "async_create handler: end\n");
        });
        return *this;
    }

    operations& start_open(int i) {
        print(PRI1); print(PRI1, "start_open(%d)\n", i);
        m_ready = false;
        async_open([this, i]() {
            print(PRI1, "async_open handler: begin\n");
            set_result_and_resume(i);
            print(PRI1, "async_open handler: end\n");
        });
        return *this;
    }

    operations& start_write(int i) {
        print(PRI1); print(PRI1, "start_write(%d)\n", i);
        m_ready = false;
        async_write([this, i]() {
            print(PRI1, "async_write handler: begin\n");
            set_result_and_resume(i);
            print(PRI1, "async_write handler: end\n");
        });
        return *this;
    }

    operations& start_read(int i) {
        print(PRI1); print(PRI1, "start_read(%d)\n", i);
        m_ready = false;
        async_read([this, i]() {
            print(PRI1, "async_read handler: begin\n");
            set_result_and_resume(i);
            print(PRI1, "async_read handler: end\n");
        });
        return *this;
    }

    operations& start_close(int i) {
        print(PRI1); print(PRI1, "start_close(%d)\n", i);
        m_ready = false;
        async_close([this, i]() {
            print(PRI1, "async_close handler: begin\n");
            set_result_and_resume(i);
            print(PRI1, "async_close handler: end\n");
        });
        return *this;
    }

    operations& start_remove(int i) {
        print(PRI1); print(PRI1, "start_remove(%d)\n", i);
        m_ready = false;
        async_remove([this, i]() {
            print(PRI1, "async_remove handler: begin\n");
            set_result_and_resume(i);
            print(PRI1, "async_remove handler: end\n");
         });
        return *this;
    }
};
