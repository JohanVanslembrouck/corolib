/**
 * @file p1200cog.cpp
 * @brief This file contains the first of three implementations of the functions 
 * start_op1_impl, start_op2_impl and start_op3_impl.
 * This implementation calls sendc_op1, sendc_op2 and sendc_op3.
 * 
 * @author Johan Vanslembrouck
 */

#include "p1200cog.h"

void RemoteObject1Co::start_op1_impl(const int idx, int in1, int in2)
{
    print(PRI2, "%p: RemoteObject1Co::start_op1_impl(%d, %d, Md)\n", this, idx, in1, in2);

    m_remoteObject.sendc_op1(in1, in2,
        [this, idx](op1_ret_t in1)
        {
            genericCompletionHandler<op1_ret_t>(idx, in1);
        });
}

void RemoteObject1Co::start_op2_impl(const int idx, int in1, int in2)
{
    print(PRI2, "%p: RemoteObject1Co::start_op2_impl(%d, %d, %d)\n", this, idx, in1, in2);

    m_remoteObject.sendc_op2(in1, in2,
        [this, idx](op2_ret_t in1)
        {
            genericCompletionHandler<op2_ret_t>(idx, in1);
        });
}

void RemoteObject1Co::start_op3_impl(const int idx, int in1)
{
    print(PRI2, "%p: RemoteObj1::start_op3_impl(%d, %d)\n", this, idx, in1);

    m_remoteObject.sendc_op3(in1,
        [this, idx](op1_ret_t in1)
        {
            genericCompletionHandler<op1_ret_t>(idx, in1);
        });
}
