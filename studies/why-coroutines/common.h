/**
 * @file common.h
 * @brief
 *
 * @author Johan Vanslembrouck
 */

#ifndef _COMMON_H_
#define _COMMON_H_

#include <functional>
#include <stdio.h>

#include "eventqueue.h"
#include "eventqueuethr.h"

// structs grouping out parameters and return value
// ------------------------------------------------

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

// lambda definitions
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

// lambda definitions using structs

using lambda_op1_ret_t = typename std::function<void(op1_ret_t)>;
using lambda_op2_ret_t = typename std::function<void(op2_ret_t)>;

// Lambda definitions using void* and structs

using lambda_vp_op1_ret_t = typename std::function<void(void*, op1_ret_t)>;
using lambda_vp_op2_ret_t = typename std::function<void(void*, op2_ret_t)>;

// event queues
// ------------

/**
 * @brief
 * Normally, we can push instances of any of the lambdas defined above onto an I/O event queue.
 * Each lambda is associated with an I/O event, which carries the arguments of the lambda.
 * When an I/O event arrives, we call the lambda with the arguments from the I/O event.
 * 
 * However, in this simplified implementation, there isn't an I/O system that can receive an I/O event.
 * Therefore, we assume that the I/O event (with its arguments) is already known and 
 * we call the lambda from an outer lambda that we push onto the event queue.
 * This outer lambda has type lambda_void_t = std::function<void(void)>.
 * 
 * At the "popping" side, the outer lambda is called, applying the inner lambda to the content of the I/O event.
 * 
 * The following is an illustration of what has just been explained.
 * 
 *      void registerCB(lambda_3int_t lambda, int in1, int in2) {
 *          eventQueue.push([lambda, in1, in2]() { lambda(1, 2, in1 + in2); });
 *                           -----------------        |
 *                              |                     |
 *                              |                     |__ inner lambds, applied to 1, 2, in1 + in2
 *                              |                         that are in a real system contained in the received I/O event
 *                              |__ outer lambda capture, with the inner lambda
 *                                                        and all parameters that allow constructing the content of the I/O event
 *                           -------------------------------------------------
 *                           outer lambda, type = std::function<void(void)>
 *      }

 * An alternative implementation in the absence of a I/O system, is to place the lambda with a corresponding I/O event
 * as one item (struct) onto an event queue and then, on the popping side, to apply the lambda to the content of the I/O event.
 * Such an implementation would require the use of e.g. a variant type.
 * The current solution uses overloaded function definitions instead.
 */

extern EventQueue eventQueue;
extern EventQueueThr<lambda_void_t, 32>  eventQueueThr;

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

// using structs grouping out parameters and return value

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

inline void startThread(lambda_void_t lambda)
{
    std::thread thread1([lambda]() {
        //printf("startThread: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        lambda();
        });
    thread1.detach();
}

// using structs grouping out parameters and return value

inline void startThread(lambda_op1_ret_t lambda, int in1, int in2)
{
    std::thread thread1([lambda, in1, in2]() {
        //printf("startThread: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        lambda({ 1, 2, in1 + in2 });
        });
    thread1.detach();
}

inline void startThread(lambda_op2_ret_t lambda, int in1, int in2)
{
    std::thread thread1([lambda, in1, in2]() {
        //printf("startThread: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        lambda({ 1, in1 + in2 });
        });
    thread1.detach();
}

// startThreadEQ functions
// -----------------------

inline void startThreadEQ(lambda_3int_t lambda, int in1, int in2)
{
    std::thread thread1([lambda, in1, in2]() {
        //printf("startThreadEQ: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        eventQueueThr.push([lambda, in1, in2]() { lambda(1, 2, in1 + in2); });
        });
    thread1.detach();
}

// using structs grouping out parameters and return value

inline void startThreadEQ(lambda_op1_ret_t lambda, int in1, int in2)
{
    std::thread thread1([lambda, in1, in2]() {
        //printf("startThreadEQ: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        eventQueueThr.push([lambda, in1, in2]() { lambda({ 1, 2, in1 + in2 }); });
        });
    thread1.detach();
}

inline void startThreadEQ(lambda_op2_ret_t lambda, int in1, int in2)
{
    std::thread thread1([lambda, in1, in2]() {
        //printf("startThreadEQ: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        eventQueueThr.push([lambda, in1, in2]() { lambda({ 1, in1 + in2 }); });
        });
    thread1.detach();
}

#include "buf+msg.h"

#endif
