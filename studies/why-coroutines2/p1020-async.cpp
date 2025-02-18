/**
 * @file p1020-async.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <assert.h>

#include "corolib/print.h"

#include "async.h"

using namespace corolib;

void on_create_complete(void* ctxt);
void on_open_complete(void* ctxt);
void on_write_complete(void* ctxt);
void on_read_complete(void* ctxt);
void on_close_complete(void* ctxt);
void on_remove_complete(void* ctxt);

struct context_t
{
    int st;
    int i;
};

void asynchronous(int st) {
    print(PRI1, "asynchronous: begin\n");
    context_t* ctxt = new context_t{ st, st + 1 };
    async_create(on_create_complete, ctxt);
    print(PRI1, "asynchronous: end\n");
}

void on_create_complete(void* ctxt) {
    context_t* pctxt = static_cast<context_t*>(ctxt);
    assert(pctxt->i == pctxt->st + 1);
    print(PRI1, "on_create_complete(): %d: open\n", pctxt->i++);
    async_open(on_open_complete, ctxt);
}

void on_open_complete(void* ctxt) {
    context_t* pctxt = static_cast<context_t*>(ctxt);
    assert(pctxt->i == pctxt->st + 2);
    print(PRI1, "on_open_complete(): %d: write\n", pctxt->i++);
    async_write(on_write_complete, ctxt);
}

void on_write_complete(void* ctxt) {
    context_t* pctxt = static_cast<context_t*>(ctxt);
    assert(pctxt->i == pctxt->st + 3);
    print(PRI1, "on_write_complete(): %d: read\n", pctxt->i++);
    async_read(on_read_complete, ctxt);
}

void on_read_complete(void* ctxt) {
    context_t* pctxt = static_cast<context_t*>(ctxt);
    assert(pctxt->i == pctxt->st + 4);
    print(PRI1, "on_read_complete(): %d: close\n", pctxt->i++);
    async_close(on_close_complete, ctxt);
}

void on_close_complete(void* ctxt) {
    context_t* pctxt = static_cast<context_t*>(ctxt);
    assert(pctxt->i == pctxt->st + 5);
    print(PRI1, "on_close_complete(): %d: remove\n", pctxt->i++);
    async_remove(on_remove_complete, ctxt);
}

void on_remove_complete(void* ctxt) {
    context_t* pctxt = static_cast<context_t*>(ctxt);
    assert(pctxt->i == pctxt->st + 6);
    print(PRI1, "on_remove_complete(): %d: done\n", pctxt->i);
    delete pctxt;
}

int main() {
    print(PRI1, "main: begin\n");
    asynchronous(10);
    asynchronous(20);
    evqueueX.run();
    print(PRI1, "main: end\n");
    return 0;
}