/**
 *  Filename: tracker1.cpp
 *  Description:
 *
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>
#include <thread>

#include "tracker1.h"

tracker1::tracker1()
{
}

tracker1::~tracker1()
{
    printf("nr_resumptions = %d\n", nr_resumptions);
    printf("Waiting 1000 milliseconds before exiting\n\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

tracker1 tracker1_obj;
