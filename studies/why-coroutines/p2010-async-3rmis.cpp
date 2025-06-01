/**
 * @file p2010-async-3rmis.cpp
 * @brief
 * This example simulates 3 asynchronous remote method invocations (RMIs). 
 * The operationX_result completion handlers are directly called from the start_operationX functions.
 * The signature of the functions stays as close as possible to the synchronous case,
 * with the start_operationX functions passing the in parameters and the  
 * the operationX_result functions passing the out parameters and the return value.
 * However, because all parameters and return values are stored in an operationContext object,
 * these parameters are not used and will be removed in the next example (p2011-async-3rmis.cpp).
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "p2000-common.h"

class Class1
{
public:
    void operationA(int in1)
    {
        operationContext* pctxt = new operationContext;
        
        printf("client part 1: Prepare input for start_operation1\n");
        pctxt->in11 = in1;
        pctxt->in12 = 2 + in1;
        printf("Calling start_operation1(in11 = %d, int11 = %d)\n", pctxt->in11,  pctxt->in12);
        start_operation1(pctxt, pctxt->in11, pctxt->in12);
    }

    void start_operation1(operationContext* pctxt, int in11, int in12)
    {
        (void)in11;
        (void)in12;
        operation1_server(pctxt);
        operation1_result(pctxt, pctxt->out11, pctxt->ret1);
    }

    void operation1_result(operationContext* pctxt, int out11, int ret1)
    {
        (void)out11;
        (void)ret1;
        printf("Returning from operation1(out11 = %d) => %d\n", pctxt->out11, pctxt->ret1);
        
        printf("client part 2: Process output from operation1 and prepare input for start_operation2\n");
        pctxt->in21 = pctxt->in11;
        pctxt->out21 = 0;
        pctxt->out22 = 0;
        
        printf("Calling start_operation2(in21 = %d)\n", pctxt->in21);
        start_operation2(pctxt, pctxt->in21);
    }

    void start_operation2(operationContext* pctxt, int in21)
    {
        (void)in21;
        operation2_server(pctxt);
        operation2_result(pctxt, pctxt->out21, pctxt->out22, pctxt->ret2);
    }

    void operation2_result(operationContext* pctxt, int out21, int out22, int ret2)
    {
        (void)out21;
        (void)out22;
        (void)ret2;
        printf("Returning from operation2(out21 = %d, out22 = %d) => %d\n", pctxt->out21, pctxt->out22, pctxt->ret2);

        printf("client part 3: Process output from operation2 and prepare input for start_operation3\n");
        pctxt->in31 = pctxt->out21;
        pctxt->inout31 = pctxt->out22;
        pctxt->out31 = pctxt->out21 + pctxt->out22;
        printf("Calling start_operation3(in31 = %d, inout31 = %d)\n", pctxt->in31, pctxt->inout31);
        start_operation3(pctxt, pctxt->in31, pctxt->inout31);
    }

    void start_operation3(operationContext* pctxt, int in31, int inout31)
    {
        (void)in31;
        (void)inout31;
        operation3_server(pctxt);
        operation3_result(pctxt, pctxt->inout31, pctxt->out31, pctxt->ret3);
    }

    void operation3_result(operationContext* pctxt, int inout31, int out31, int ret3)
    {
        (void)inout31;
        (void)out31;
        (void)ret3;
        printf("Returning from operation3(inout31 = %d, out31 = %d) => %d\n", pctxt->inout31, pctxt->out31, pctxt->ret3);

        printf("client part 4: Process output from operation3\n");
        pctxt->out1 = pctxt->inout31 + pctxt->out31;
        pctxt->ret = pctxt->ret1 + pctxt->ret2 + pctxt->ret3;
        printf("out1 = %d, ret = %d\n", pctxt->out1, pctxt->ret);
        
        delete pctxt;
    }
};

int main()
{
    Class1 obj1;
    obj1.operationA(10);
    obj1.operationA(20);
    return 0;
}
