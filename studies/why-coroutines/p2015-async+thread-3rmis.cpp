/**
 * @file p2015-async+thread-3rmis.cpp
 * @brief
 * This example is an evolution of p2013-async-3rmis.cpp.
 * It uses a thread to restore the sequential flow of p2000-sync-3rmis.cpp.
 * 
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>
#include <semaphore>

#include "p2000-common.h"

class Class1
{
public:
    void operationA(int in1)
    {
        operationContext* ctxt = new operationContext;
		
		printf("client part 1: Prepare input for start_operation1\n");
        ctxt->in11 = in1;
        ctxt->in12 = 2 + in1;
		printf("Calling start_operation1(): ctxt->in11 = %d, ctxt->int12 = %d\n", ctxt->in11,  ctxt->in12);
        lambda_vp_t l1 = [this](void* ctxt) { this->operation1_result(ctxt); };
        start_operation1(l1, ctxt);
        await();

		printf("client part 2: Process output from operation1 and prepare input for operation2\n");
        ctxt->in21 = ctxt->in11;
        ctxt->out21 = 0;
        ctxt->out22 = 0;
		printf("Calling start_operation2(): ctxt->in21 = %d\n", ctxt->in21);
        lambda_vp_t l2 = [this](void* ctxt) { this->operation2_result(ctxt); };
        start_operation2(l2, ctxt);
        await();

		printf("client part 3: Process output from operation2 and prepare input for operation3\n");
        ctxt->in31 = ctxt->out21;
        ctxt->inout31 = ctxt->out22;
        ctxt->out31 = 0;
		printf("Calling start_operation3(): ctxt->in31 = %d, ctxt->inout31 = %d\n", ctxt->in31, ctxt->inout31);
        lambda_vp_t l3 = [this](void* ctxt) { this->operation3_result(ctxt); };
        start_operation3(l3, ctxt);
        await();

		printf("client part 4: Process output from operation3\n");
        ctxt->out1 = ctxt->inout31 + ctxt->out31;
        ctxt->ret = ctxt->ret1 + ctxt->ret2 + ctxt->ret3;
        printf("out1 = %d, ret = %d\n", ctxt->out1, ctxt->ret);
    }

    void start_operation1(lambda_vp_t lambda, void* pctxt)
    {
        operationContext* ctxt = static_cast<operationContext*>(pctxt);
		operation1_server(ctxt);
        startThread(lambda, ctxt);
    }

    void operation1_result(void* pctxt)
    {
        operationContext* ctxt = static_cast<operationContext*>(pctxt);
		printf("Returning from operation1(out11 = %d) => %d\n", ctxt->out11, ctxt->ret1);
        m_binsema.release();
    }

    void start_operation2(lambda_vp_t lambda, void* pctxt)
    {
        operationContext* ctxt = static_cast<operationContext*>(pctxt);
		operation2_server(ctxt);
        startThread(lambda, ctxt);
    }

    void operation2_result(void* pctxt)
    {
        operationContext* ctxt = static_cast<operationContext*>(pctxt);
		printf("Returning from operation2(out21 = %d, out22 = %d) => %d\n", ctxt->out21, ctxt->out22, ctxt->ret2);
        m_binsema.release();
    }

    void start_operation3(lambda_vp_t lambda, void* pctxt)
    {
        operationContext* ctxt = static_cast<operationContext*>(pctxt);
		operation3_server(ctxt);
        startThread(lambda, ctxt);
    }

    void operation3_result(void* pctxt)
    {
        operationContext* ctxt = static_cast<operationContext*>(pctxt);
		printf("Returning from operation3(inout31 = %d, out31 = %d) => %d\n", ctxt->inout31, ctxt->out31, ctxt->ret3);
        m_binsema.release();
    }
	
	void await()
    {
        m_binsema.acquire();
    }

    std::binary_semaphore m_binsema{ 0 };
};

int main()
{
    Class1 obj1;
    obj1.operationA(10);
	obj1.operationA(20);
    return 0;
}