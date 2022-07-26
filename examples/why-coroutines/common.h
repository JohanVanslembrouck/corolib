/**
 * @file common.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include <functional>

#include "eventqueue.h"

typedef std::function<void(int, int, int)> lambda1;
typedef std::function<void(int, int)> lambda2;
typedef std::function<void(void)> lambda3;

void connect(int i, lambda3 l)
{
    printf("connect(%d, l) - begin\n", i);
    eventQueue.push([l]() { l();  });
    printf("connect(%d, l) - end\n", i);
}

int start_time;
int get_current_time() { return 0; }
int elapsed_time;

#endif

