/**
 *  Filename: print.cpp
 *  Description:
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include "print.h"

uint64_t threadids[128];

static int get_thread_number64(uint64_t id)
{
    for (int i = 0; i < 128; i++)
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

static int get_thread_number32(uint32_t id)
{
    for (int i = 0; i < 128; i++)
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

static uint64_t get_thread_id()
{
    auto id = std::this_thread::get_id();
    uint64_t* ptr = (uint64_t*)&id;
    return (uint64_t)(*ptr);
}

void print(int pri, const char* fmt, ...)
{
    va_list arg;
    if (print_priority & pri)
    {
        char msg[512];

        va_start(arg, fmt);
#if defined(_WIN32)
        int n = vsprintf_s(msg, fmt, arg);
#else
        int n = vsprintf(msg, fmt, arg);
#endif
        va_end(arg);

        (void)n;
        int threadid = (sizeof(std::thread::id) == sizeof(uint32_t)) ?
            get_thread_number32((uint32_t)get_thread_id()) :
            get_thread_number64(get_thread_id());
        fprintf(stderr, "%02d: %s", threadid, msg);
    }
}

int print_priority = 0x01;
