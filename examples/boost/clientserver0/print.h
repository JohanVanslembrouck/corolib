/**
 *  @file print.h
 *  @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#pragma once

using namespace std;

#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <string>
#include <thread>

/**
 * A tailored print function that first prints a logical thread id (0, 1, 2, ...)
 * before printing the original message.
 *
 */

const int PRI1 = 0x01;
const int PRI2 = 0x02;
const int PRI3 = 0x04;
const int PRI4 = 0x08;

extern int priority;

int get_thread_number(uint64_t id);
int get_thread_number64(uint64_t id);
int get_thread_number32(uint32_t id);

uint64_t get_thread_id();

void print();
void print(const char* fmt, ...);
void print(int pri, const char* fmt, ...);
