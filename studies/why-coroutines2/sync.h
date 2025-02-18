/**
 * @file sync.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#pragma once

#include "corolib/print.h"

using namespace corolib;

int create(int i) { print(PRI1, "create()\n"); return i; }
int open(int i )  { print(PRI1, "open()\n");   return i; }
int write(int i ) { print(PRI1, "write()\n");  return i; }
int read(int i )  { print(PRI1, "read()\n");   return i; }
int close(int i ) { print(PRI1, "close()\n");  return i; }
int remove(int i) { print(PRI1, "remove()\n"); return i; }
