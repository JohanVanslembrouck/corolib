/**
 * @file async.h
 * @brief 
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#pragma once

#include "corolib/print.h"

#include "eventqueue.h"

using namespace corolib;

void async_create(FunctionVoidVoid f) { print(PRI1, "async_create()\n"); evqueue.push(f); }
void async_open(FunctionVoidVoid f)   { print(PRI1, "async_open()\n");   evqueue.push(f); }
void async_write(FunctionVoidVoid f)  { print(PRI1, "async_write()\n");  evqueue.push(f); }
void async_read(FunctionVoidVoid f)   { print(PRI1, "async_read()\n");   evqueue.push(f); }
void async_close(FunctionVoidVoid f)  { print(PRI1, "async_close()\n");  evqueue.push(f); }
void async_remove(FunctionVoidVoid f) { print(PRI1, "async_remove()\n"); evqueue.push(f); }

void async_create(FunctionVoidVoidPtr f, void* ctxt) { print(PRI1, "async_create(%d)\n", 0); evqueueX.push({ f, ctxt }); }
void async_open(FunctionVoidVoidPtr f, void* ctxt)   { print(PRI1, "async_open(%d)\n", 0);   evqueueX.push({ f, ctxt }); }
void async_write(FunctionVoidVoidPtr f, void* ctxt)  { print(PRI1, "async_write(%d)\n", 0);  evqueueX.push({ f, ctxt }); }
void async_read(FunctionVoidVoidPtr f, void* ctxt)   { print(PRI1, "async_read(%d)\n", 0);   evqueueX.push({ f, ctxt }); }
void async_close(FunctionVoidVoidPtr f, void* ctxt)  { print(PRI1, "async_close(%d)\n", 0);  evqueueX.push({ f, ctxt }); }
void async_remove(FunctionVoidVoidPtr f, void* ctxt) { print(PRI1, "async_remove(%d)\n", 0); evqueueX.push({ f, ctxt }); }
