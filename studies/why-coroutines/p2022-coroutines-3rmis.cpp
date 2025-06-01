/**
 * @file p2022-coroutines-3rmis.cpp
 * @brief
 * This example uses coroutines.
 * The operationX coroutines use in and reference (out) variables as coroutine parameters.
 * This example uses asynchronous API sendc_operationX functions that place a completion handler in the event queue.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */
 
#include <stdio.h>

#include <corolib/print.h>
#include <corolib/commservice.h>
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

using lambda_2int_t = typename std::function<void(int, int)>;
using lambda_3int_t = typename std::function<void(int, int, int)>;

class Class1 : public CommService
{
private:
    void sendc_operation1(int in11, int in12, lambda_2int_t lambda)
    {
        int out11;
        int ret1;
        operation1_server(in11, in12, out11, ret1);
        eventQueue.push([lambda, out11, ret1]() { lambda(out11, ret1); });
    }
    
    void sendc_operation2(int in21, lambda_3int_t lambda)
    {
        int out21;
        int out22;
        int ret2;
        operation2_server(in21, out21, out22, ret2);
        eventQueue.push([lambda, out21, out22, ret2]() { lambda(out21, out22, ret2); });
    }
    
    void sendc_operation3(int in31, int inout31, lambda_3int_t lambda)
    {
        int out31;
        int ret3;
        operation3_server(in31, inout31, out31, ret3);
        eventQueue.push([lambda, inout31, out31, ret3]() { lambda(inout31, out31, ret3); });
    }
    
public:
    async_task<int> operationA(int in1, int& out1)
    {
        printf("client part 1: Prepare input for operation1\n");
        int in11 = in1;
        int in12 = 2 + in1;
        int out11 = 0;
        printf("Calling operation1(in11 = %d, int12 = %d, out11 = %d) => %d\n", in11, in12, out11, 0);
        int ret1 = co_await operation1(in11, in12, out11);

        printf("client part 2: Process output from operation1 and prepare input for operation2\n");
        int in21 = in11;
        int out21 = 0;
        int out22 = 0;
        printf("Calling operation2(in21 = %d, out21 = %d, out22 = %d) => %d\n", in21, out21, out22, 0);
        int ret2 = co_await operation2(in21, out21, out22);

        printf("client part 3: Process output from operation2 and prepare input for operation3\n");
        int in31 = out21;
        int inout31 = out22;
        int out31 = 0;
        printf("Calling operation3(in31 = %d, inout31 = %d, out31 = %d) => %d\n", in31, inout31, out31, 0);
        int ret3 = co_await operation3(in31, inout31, out31);

        printf("client part 4: Process output from operation3\n"); 
        out1 = inout31 + out31;
        int res = ret1 + ret2 + ret3;
        co_return res;
    }

    async_task<int> operation1(int in11, int in12, int& out11)
    {
        int2_t res = co_await start_operation1(in11, in12);
        out11 = res.el1;
        int ret = res.el2;
        printf("Returning from operation1(in11 = %d, int12 = %d, out11 = %d) => %d\n", in11, in12, out11, ret);
        co_return ret;
    }

    async_operation<int2_t> start_operation1(int in11, int in12)
    {
        int index = get_free_index();
        sendc_operation1(in11, in12,
            [this, index](int out11, int ret1)
            {
                int2_t op1_ret = { out11, ret1 };
                this->completionHandler<int2_t>(index, op1_ret);
            });
        return { this, index };
    }

    async_task<int> operation2(int in21, int& out21, int& out22)
    {
        int3_t res = co_await start_operation2(in21);
        out21 = res.el1;
        out22 = res.el2;
        int ret = res.el3;
        printf("Returning from operation2(in21 = %d, out21 = %d, out22 = %d) => %d\n", in21, out21, out22, ret);
        co_return ret;
    }
    
    async_operation<int3_t> start_operation2(int in21)
    {
        int index = get_free_index();
        sendc_operation2(in21,
            [this, index](int out21, int out22, int ret2)
            {
                int3_t op2_ret = { out21, out22, ret2 };
                this->completionHandler<int3_t>(index, op2_ret);
            });
        return { this, index };
    }

    async_task<int> operation3(int in31, int& inout31, int& out31)
    {
        int3_t res = co_await start_operation3(in31, inout31);
        inout31 = res.el1;
        out31 = res.el2;
        int ret = res.el3;
        printf("Returning from operation3(in31 = %d, inout31 = %d, out31 = %d) => %d\n", in31, inout31, out31, ret);
        co_return ret;
    }
    
    async_operation<int3_t> start_operation3(int in31, int inout31)
    {
        int index = get_free_index();
        sendc_operation3(in31, inout31,
            [this, index](int inout31, int out31, int ret)
            {
                int3_t op3_ret = { inout31, out31, ret };
                this->completionHandler<int3_t>(index, op3_ret);
            });
        return { this, index };
    }
};

EventQueue eventQueue;

int main()
{
    Class1 obj1;
    int out1 = 0;
    int out2 = 0;
    async_task<int> t1 = obj1.operationA(10, out1);
    async_task<int> t2 = obj1.operationA(20, out2);
    printf("-- before eventQueue.run();\n");
    eventQueue.run();
    printf("-- after eventQueue.run();\n");
    int ret1 = t1.get_result();
    int ret2 = t2.get_result();
    printf("out1 = %d, ret1 = %d\n", out1, ret1);
    printf("out2 = %d, ret2 = %d\n", out2, ret2);
    return 0;
}