/**
 * @file asyncthr.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#pragma once

#include "corolib/print.h"

#include "eventqueuethr.h"

using namespace corolib;

void async_create(FunctionVoidVoid f) { print(PRI1, "async_create()\n"); evqueuethr.push(f);  }
void async_open(FunctionVoidVoid f)   { print(PRI1, "async_open()\n");   evqueuethr.push(f);  }
void async_write(FunctionVoidVoid f)  { print(PRI1, "async_write()\n");  evqueuethr.push(f);  }
void async_read(FunctionVoidVoid f)   { print(PRI1, "async_read()\n");   evqueuethr.push(f);  }
void async_close(FunctionVoidVoid f)  { print(PRI1, "async_close()\n");  evqueuethr.push(f);  }
void async_remove(FunctionVoidVoid f) { print(PRI1, "async_remove()\n"); evqueuethr.push(f);  }
