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
#include <string>
#include <thread>

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

	uint64_t threadids[NR_THREADS];

	int get_thread_number64(uint64_t id)
	{
		for (int i = 0; i < NR_THREADS; i++)
		{
			if (threadids[i] == id)
				return i;
			if (threadids[i] == 0) {
				threadids[i] = id;
				return i;
			}
		}
		return -1;
	}

	int get_thread_number32(uint32_t id)
	{
		for (int i = 0; i < NR_THREADS; i++)
		{
			if (threadids[i] == id)
				return i;
			if (threadids[i] == 0)
			{
				threadids[i] = id;
				return i;
			}
		}
		return -1;
	}

	uint64_t get_thread_id()
	{
		auto id = std::this_thread::get_id();
		uint64_t* ptr = (uint64_t*)&id;
		return (uint64_t)(*ptr);
	}

	extern const int priority;

	void print(int pri, const char* fmt, ...)
	{
		va_list arg;
		if (priority & pri)
		{
			char msg[512];

			va_start(arg, fmt);
#if defined(_WIN32)
			int n = vsprintf_s(msg, fmt, arg);
#else
			int n = vsprintf(msg, fmt, arg);
#endif
			va_end(arg);

			int threadid = (sizeof(std::thread::id) == sizeof(uint32_t)) ?
				get_thread_number32((uint32_t)get_thread_id()) :
				get_thread_number64(get_thread_id());
			fprintf(stderr, "%02d: %s", threadid, msg);
		}
	}
}

#endif
