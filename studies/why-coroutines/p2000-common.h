/**
 * @file p2000-common.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */
 
#ifndef _P2000_COMMON_H_
#define _P2000_COMMON_H_
 
#include <stdio.h>
#include <thread>
#include <functional>

#include "eventqueue.h"

extern EventQueue eventQueue;

// -------------------------------------------------------------
// Functions independent of application types
// -------------------------------------------------------------

using lambda_vp_t = typename std::function<void(void*)>;

inline void registerCB(lambda_vp_t lambda, void* val)
{
    //printf("registerCB(lambda_bool_t lambda, %d)\n", val);
    eventQueue.push([lambda, val]() { lambda(val); });
}

inline void startThread(lambda_vp_t lambda, void* val)
{
	std::thread thread1([lambda, val]() {
		//printf("startThread: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", 100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		lambda(val);
    });
    thread1.detach();
}

// -------------------------------------------------------------
// Server side functions
// -------------------------------------------------------------

void operation1_server(int in11, int in12, int& out11, int& ret1)
{
	printf("operation1_server: calculate out parameters and return value\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
    out11 = in11 + in12;
    ret1 = 2 + out11;
}

void operation2_server(int in21, int& out21, int& out22, int& ret2)
{
	printf("operation2_server: calculate out parameters and return value\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
    out21 = in21;
    out22 = 2 + in21;
    ret2 = 3 + in21;
}

void operation3_server(int in31, int& inout31, int& out31, int& ret3)
{
	printf("operation3_server: Calculate out parameters and return value\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
    inout31 = in31;
    out31 = 2 + in31;
    ret3 = 3 + in31;
}

// -------------------------------------------------------------
// Context type holding all parameters and return values
// -------------------------------------------------------------

struct operationContext
{
	int in1 = 0;
	int out1 = 0;

	int in11 = 0;
	int in12 = 0;
	int out11 = 0;
	int ret1 = 0;

	int in21 = 0;
	int out21 = 0;
	int out22 = 0;
	int ret2 = 0;

	int in31 = 0;
	int inout31 = 0;
	int out31 = 0;
	int ret3 = 0;
	int ret = 0;
};

// -------------------------------------------------------------
// Functions dependent on the application types
// -------------------------------------------------------------

using lambda_operationContext_t = typename std::function<void(operationContext*)>;

inline void registerCB2(lambda_operationContext_t lambda, operationContext* val)
{
    //printf("registerCB(lambda_bool_t lambda, %d)\n", val);
    eventQueue.push([lambda, val]() { lambda(val); });
}

inline void startThread2(lambda_operationContext_t lambda, operationContext* val)
{
	std::thread thread1([lambda, val]() {
		//printf("startThread: std::this_thread::sleep_for(std::chrono::milliseconds(%d));\n", 100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		lambda(val);
    });
    thread1.detach();
}

// -------------------------------------------------------------
// Server side functions, using operationContext* as only parameter
// -------------------------------------------------------------

void operation1_server(operationContext* ctxt)
{
	printf("operation1_server: calculate out parameters and return value\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ctxt->out11 = ctxt->in11 + ctxt->in12;
    ctxt->ret1 = 2 + ctxt->out11;
}

void operation2_server(operationContext* ctxt)
{
	printf("operation2_server: calculate out parameters and return value\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ctxt->out21 = ctxt->in21;
    ctxt->out22 = 2 + ctxt->in21;
    ctxt->ret2 = 3 + ctxt->in21;
}

void operation3_server(operationContext* ctxt)
{
	printf("operation3_server: Calculate out parameters and return value\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ctxt->inout31 = ctxt->in31;
    ctxt->out31 = 2 + ctxt->in31;
    ctxt->ret3 = 3 + ctxt->in31;
}

#endif
