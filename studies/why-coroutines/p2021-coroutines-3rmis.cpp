/**
 * @file p2021-coroutines-3rmis.cpp
 * @brief
 * This example uses coroutines.
 * This variant of p2020-coroutines-3rmis.cpp does not use
 * reference (out) variables as coroutine parameters.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */
 
#include <stdio.h>

#include <corolib/print.h>
#include <corolib/async_task.h>
#include <corolib/async_operation.h>

#include "p2000-common.h"

using namespace corolib;

struct int2_t
{
    int el1;
    int el2;
};

struct int3_t
{
    int el1;
    int el2;
    int el3;
};

class Class1
{
public:
    async_task<int> operationA(int in1, int& out1)
    {
		printf("client part 1: Prepare input for operation1\n");
        int in11 = in1;
        int in12 = 2 + in1;
		printf("Calling operation1(in11 = %d, int12 = %d)\n", in11, in12);
        int2_t ret1val = co_await operation1(in11, in12);
        int out11 = ret1val.el1;
        (void)out11;
        int ret1 = ret1val.el2;

		printf("client part 2: Process output from operation1 and prepare input for operation2\n");
        int in21 = in11;
        
		printf("Calling operation2(in21 = %d)\n", in21);
        int3_t ret2val = co_await operation2(in21);
        int out21 = ret2val.el1;
        int out22 = ret2val.el2;
        int ret2 = ret2val.el3;

		printf("client part 3: Process output from operation2 and prepare input for operation3\n");
        int in31 = out21;
        int inout31 = out22;
		printf("Calling operation3(in31 = %d, inout31 = %d)\n", in31, inout31);
        int3_t ret3val = co_await operation3(in31, inout31);
        inout31 = ret3val.el1;
        int out31 = ret3val.el2;
        int ret3 = ret3val.el3;

		printf("client part 4: Process output from operation3\n"); 
        out1 = inout31 + out31;
        int res = ret1 + ret2 + ret3;
        co_return res;
    }

    async_task<int2_t> operation1(int in11, int in12)
    {
		int ret;
        int out11;
		operation1_server(in11, in12, out11, ret);
        printf("Returning from operation1(in11 = %d, int12 = %d, out11 = %d) => %d\n", in11, in12, out11, ret);
        co_return{ out11, ret };
    }

    async_task<int3_t> operation2(int in21)
    {
		int ret;
        int out21;
        int out22;
		operation2_server(in21, out21, out22, ret);
        printf("Returning from operation2(in21 = %d, out21 = %d, out22 = %d) => %d\n", in21, out21, out22, ret);
        co_return{ out21, out22, ret };
    }

    async_task<int3_t> operation3(int in31, int inout31)
    {
		int ret;
        int out31;
        int inout31a = inout31;
		operation3_server(in31, inout31a, out31, ret);
        printf("Returning from operation3(in31 = %d, inout31a = %d, out31 = %d) => %d\n", in31, inout31a, out31, ret);
        co_return{ inout31a, out31, ret };
    }
};

int main()
{
    Class1 obj1;
    int out1 = 0;
	int out2 = 0;
    async_task<int> t1 = obj1.operationA(10, out1);
	async_task<int> t2 = obj1.operationA(20, out2);
    int ret1 = t1.get_result();
	int ret2 = t2.get_result();
    printf("out1 = %d, ret1 = %d\n", out1, ret1);
	printf("out2 = %d, ret2 = %d\n", out2, ret2);
    return 0;
}