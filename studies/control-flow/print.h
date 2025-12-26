/**
 * @file print.h
 * @brief
 *
 * @author Johan Vanslembrouck
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

extern int print_priority;

inline void set_print_level(int pri) { print_priority = pri; }

void print(int pri, const char* fmt, ...);
