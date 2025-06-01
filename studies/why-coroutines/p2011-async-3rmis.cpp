/**
 * @file p2011-async-3rmis.cpp
 * @brief
 * This example simulates 3 asynchronous remote method invocations (RMIs). 
 * The operationX_result completion handlers are directly called from the start_operationX functions.
 * The difference with p2010-async-3rmis.cpp is that the start_operationX and operationX_result
 * functions take operationContext* as sole parameter. They have the same signature.
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
        
        printf("Client part 1: Prepare input for start_operation1\n");
        pctxt->in11 = in1;
        pctxt->in12 = 2 + in1;
        printf("Calling start_operation1(): pctxt->in11 = %d, pctxt->int12 = %d\n", pctxt->in11,  pctxt->in12);
        start_operation1(pctxt);
    }

    void start_operation1(operationContext* pctxt)
    {
        operation1_server(pctxt);
        operation1_result(pctxt);
    }

    void operation1_result(operationContext* pctxt)
    {
        printf("Returning from operation1(out11 = %d) => %d\n", pctxt->out11, pctxt->ret1);
        
        printf("Client part 2: Process output from operation1 and prepare input for start_operation2\n");  
        pctxt->in21 = pctxt->in11;
        pctxt->out21 = 0;
        pctxt->out22 = 0;
        printf("Calling start_operation2(): pctxt->in21 = %d\n", pctxt->in21);
        start_operation2(pctxt);
    }

    void start_operation2(operationContext* pctxt)
    {
        operation2_server(pctxt);
        operation2_result(pctxt);
    }

    void operation2_result(operationContext* pctxt)
    {
        printf("Returning from operation2(out21 = %d, out22 = %d) => %d\n", pctxt->out21, pctxt->out22, pctxt->ret2);

        printf("Client part 3: Process output from operation2 and prepare input for start_operation3\n");
        pctxt->in31 = pctxt->out21;
        pctxt->inout31 = pctxt->out22;
        pctxt->out31 = 0;
        printf("Calling start_operation3(): pctxt->in31 = %d, pctxt->inout31 = %d\n", pctxt->in31, pctxt->inout31);
        start_operation3(pctxt);
    }

    void start_operation3(operationContext* pctxt)
    {
        operation3_server(pctxt);
        operation3_result(pctxt);
    }

    void operation3_result(operationContext* pctxt)
    {    
        printf("Returning from operation3(inout31 = %d, out31 = %d) => %d\n", pctxt->inout31, pctxt->out31, pctxt->ret3);

        printf("Client part 4: Process output from operation3\n");
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