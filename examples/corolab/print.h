#pragma once

using namespace std;

#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <string>

uint64_t threadids[128];

int get_thread_number(uint64_t id)
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

uint64_t get_thread_id()
{
    //static_assert(sizeof(std::thread::id) == sizeof(uint64_t), 
    //    "this function only works if size of thead::id is equal to the size of uint_64");
    auto id = std::this_thread::get_id();
    uint64_t* ptr = (uint64_t*)& id;
    return (*ptr);
}

void print(const char* fmt, ...)
{
    va_list arg;
    char msg[256];

    va_start(arg, fmt);
    int n = vsprintf_s(msg, fmt, arg);
    va_end(arg);

    int threadid = get_thread_number(get_thread_id());
    fprintf(stderr, "%02d: %s", threadid, msg);
}
