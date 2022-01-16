/**
 * @file print.h
 * @brief
 * A tailored print function that first prints a logical thread id (0, 1, 2, ...)
 * before printing the original message.
 * print takes a first argument (pri) that allows defining several groups (priorities)
 * of messages that will or will not be printed depending on the priority variable
 * defined in the application.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@altran.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _PRINT_H_
#define _PRINT_H_

#include <stdio.h>
#include <stdarg.h>

namespace corolib
{
    const int PRI1 = 0x01;
    const int PRI2 = 0x02;
    const int PRI3 = 0x04;
    const int PRI4 = 0x08;
    const int PRI5 = 0x10;
    const int PRI6 = 0x20;
    const int PRI7 = 0x40;
    const int PRI8 = 0x80;

    const int NR_THREADS = 128;

    void set_priority(int pri);

	inline void print() { fprintf(stderr, "\n"); }
    void print(int pri, const char* fmt, ...);
}

#endif
