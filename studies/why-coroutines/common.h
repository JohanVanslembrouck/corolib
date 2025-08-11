/**
 * @file common.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include <functional>
#include <stdio.h>

#include "eventqueue.h"

// Lambda definitions
// ------------------

using lambda_3int_t = typename std::function<void(int, int, int)>;
using lambda_2int_t = typename std::function<void(int, int)>;
using lambda_1int_t = typename std::function<void(int)>;
using lambda_bool_t = typename std::function<void(bool)>;
using lambda_void_t = typename std::function<void(void)>;

using lambda_vp_3int_t = typename std::function<void(void*, int, int, int)>;
using lambda_vp_2int_t = typename std::function<void(void*, int, int)>;
using lambda_vp_1int_t = typename std::function<void(void*, int)>;
using lambda_vp_bool_t = typename std::function<void(void*, bool)>;
using lambda_vp_t = typename std::function<void(void*)>;


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

using lambda_vp_op1_ret_t = typename std::function<void(void*, op1_ret_t)>;
using lambda_vp_op2_ret_t = typename std::function<void(void*, op2_ret_t)>;


// There isn't an I/O system that will place the lambda in the event queue
// when an I/O event arrives. Therefore we do it ourselves.

extern EventQueue eventQueue;

// registerCB functions
// --------------------

inline void registerCB(lambda_3int_t lambda, int in1, int in2)
{
    //printf("registerCB(lambda_3int_t lambda, %d, %d)\n", in1, in2);
    eventQueue.push([lambda, in1, in2]() { lambda(1, 2, in1 + in2); });
}

inline void registerCB(lambda_2int_t lambda, int in1, int in2)
{
    //printf("registerCB(lambda_2int_t lambda, %d, %d)\n", in1, in2);
    eventQueue.push([lambda, in1, in2]() { lambda(1, in1 + in2); });
}

inline void registerCB(lambda_void_t lambda)
{
    //printf("registerCB(lambda_void_t lambda)\n");
    eventQueue.push(lambda);
}

inline void registerCB(lambda_op1_ret_t lambda, int in1, int in2)
{
    //printf("registerCB(lambda_op1_ret_t lambda, %d, %d)\n", in1, in2);
    eventQueue.push([lambda, in1, in2]() { lambda({ 1, 2, in1 + in2 }); });
}

inline void registerCB(lambda_op2_ret_t lambda, int in1, int in2)
{
    //printf("registerCB(lambda_op2_ret_t lambda, %d, %d)\n", in1, in2);
    eventQueue.push([lambda, in1, in2]() { lambda({ 1, in1 + in2 }); });
}

inline void registerCB(lambda_bool_t lambda, bool val)
{
    //printf("registerCB(lambda_bool_t lambda, %d)\n", val);
    eventQueue.push([lambda, val]() { lambda(val); });
}

inline void registerCB(lambda_vp_3int_t lambda, void* context, int in1, int in2)
{
    //printf("registerCB(lambda_vp_3int_t lambda, void* context, %d, %d)\n", in1, in2);
    eventQueue.push([lambda, context, in1, in2]() { lambda(context, 1, 2, in1 + in2); });
}

// startThread functions
// ---------------------

inline void startThread(lambda_3int_t lambda, int in1, int in2)
{
    std::thread thread1([lambda, in1, in2]() {
        //printf("startThread: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        lambda(1, 2, in1 + in2);
        });
    thread1.detach();
}

inline void startThread(lambda_2int_t lambda, int in1, int in2)
{
    std::thread thread1([lambda, in1, in2]() {
        //printf("startThread: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        lambda(1, in1 + in2);
        });
    thread1.detach();
}

inline void startThread(lambda_3int_t lambda, int in1)
{
    std::thread thread1([lambda, in1]() {
        //printf("startThread: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        lambda(1, 2, in1);
        });
    thread1.detach();
}

#include "buf+msg.h"

inline void startThread(lambda_void_t lambda)
{
    std::thread thread1([lambda]() {
        //printf("startThread: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        lambda();
        });
    thread1.detach();
}

#endif
