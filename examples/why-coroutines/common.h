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

typedef std::function<void(int, int, int)>  lambda_3int_t;
typedef std::function<void(int, int)>       lambda_2int_t;
typedef std::function<void(int)>            lambda_1int_t;
typedef std::function<void(void)>           lambda_void_t;

void connect(int i, lambda_void_t l)
{
    printf("connect(%d, l) - begin\n", i);
    eventQueue.push([l]() { l();  });
    printf("connect(%d, l) - end\n", i);
}

int start_time;
int get_current_time() { return 0; }
int elapsed_time;

#endif

