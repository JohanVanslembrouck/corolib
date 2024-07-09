/**
 *  Filename: tracker1.h
 *  Description:
 *  Class to count the number of resumv() calls.
 * 
 *  Author: Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _TRACKER1_H_
#define _TRACKER1_H_

struct tracker1
{
    tracker1();
    ~tracker1();

    int nr_resumptions = 0;
};

extern tracker1 tracker1_obj;

#endif