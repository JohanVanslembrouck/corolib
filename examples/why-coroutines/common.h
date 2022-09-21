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

using lambda_3int_t = typename std::function<void(int, int, int)>;
using lambda_2int_t = typename std::function<void(int, int)>;
using lambda_1int_t = typename std::function<void(int)>;
using lambda_bool_t = typename std::function<void(bool)>;
using lambda_void_t = typename std::function<void(void)>;

struct op1_ret_t
{
    int out1;
    int out2;
    int ret;
};

struct op2_ret_t
{
    int out1;
    int ret;
};

using lambda_op1_ret_t = typename std::function<void(op1_ret_t)>;
using lambda_op2_ret_t = typename std::function<void(op2_ret_t)>;

int start_time;
int get_current_time() { return 0; }
int elapsed_time;

#endif
