/**
 * @file common.h
 * @brief See below for explanation.
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

/**
 * @brief An asynchronous communication framework (ACF) starts a non-blocking operation,
 * typically passing the in-parameters of the operation and a callback function (CB)
 * that the ACF will call when then operation commplets.
 * The CB will fill in the out parameters and return value of the equivalent synchronous operation.
 * Also, the CB will start a new operation, or if called on a dedicated thread,
 * unblock the thread that launched the operation and waits for it completion.
 * In the case of coroutines, the CB will resume the coroutine that co_awaits the operation.
 * 
 * We will use function objects as callback functions.
 * This file first defines all function objects that will be used by the examples in this folder.
 * These function ohjects are called lambdas.
 * Then, we will define 3 mechanisms to use these lambdas and the arguments they need,
 * identified by 3 overloaded functions:
 * 1. registerCB
 * 2. startThread
 * 3. startThreadEQ
 * See the description of each function for further explanation.
 */

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

/**
 * @brief
 * registerCB functions (where CB stands for callback) construct an function object of type std::function<void(void)>
 * and place this function object onto a queue.
 * The first parameter of aa registerCB function is a lambda.
 * The next parameters (if any) are the arguments of the lambda.
 * 
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
 * It is important to do this on the popping side, because (in the context of coroutines),
 * this will typically resume a suspended coroutine.
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
 * In the example above { 1, 2, in1 + in2 } is passed as the content of the I/O event, together with lambda_3int_t lambda1.
 * At the popping side, we apply lambda1 to the event: lambda1(1, 2, in1 + in2);
 * 
 * Such an implementation would require the use of e.g. a variant type.
 * The current solution uses overloaded function definitions instead.
 * 
 * registerCB functions are used to simulate communication frameworks
 * that (must) enter an event loop to process the completion of asynchronous operations.
 * Such communication frameworks are often single-threaded.
 */

extern EventQueue eventQueue;

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

/**
 * @brief
 * startThread functions start a thread and apply the lambda to its arguments in this thread.
 * Because the lambdas typically resume a suspended coroutine, the coroutine will be resumed
 * on the thread launched in the startThread function.
 * 
 * startThread functions are used to simulate communication frameworks 
 * where an asynchronous operation completes on a dedicated thread.
 */

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

/**
 * @brief
 * startThreadEQ functions (where EQ stands for event queue) combine functionality of startThread and registerCB.
 * 
 * startThreadEQ functions are used to simulate communication frameworks 
 * where an asynchronous operation completes on a dedicated thread,
 * however, we want all application code to run a single thread.
 * 
 * However, instead of calling the completion function directly on the thread as in startThread,
 * we construct an outer lambda and place this one on an event queue as in registerCB.
 * This approach allows running all application code on a single thread.
 * 
 * Notice that this event queue has to be thread-safe because pushing and popping is done on a different thread.
 */

extern EventQueueThr<lambda_void_t, 32>  eventQueueThr;

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
