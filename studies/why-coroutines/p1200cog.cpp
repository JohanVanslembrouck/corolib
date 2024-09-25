/**
 * @file p1200cog.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include "p1200cog.h"

void RemoteObject1Co::start_op1_impl(const int idx, int in1, int in2)
{
    print(PRI1, "%p: RemoteObject1Co::start_op1_impl(%d)\n", this, idx);

    sendc_op1(in1, in2,
        [this, idx](op1_ret_t in1)
        {
            genericCompletionHandler<op1_ret_t>(idx, in1);
        });
}

void RemoteObject1Co::start_op2_impl(const int idx, int in1, int in2)
{
    print(PRI1, "%p: RemoteObject1Co::start_op2_impl(%d)\n", this, idx);

    sendc_op2(in1, in2,
        [this, idx](op2_ret_t in1)
        {
            genericCompletionHandler<op2_ret_t>(idx, in1);
        });
}

void RemoteObject1Co::start_op3_impl(const int idx, int in1)
{
    print(PRI1, "%p: RemoteObj1::start_op3_impl(%d)\n", this, idx);

    sendc_op3(in1,
        [this, idx](op1_ret_t in1)
        {
            genericCompletionHandler<op1_ret_t>(idx, in1);
        });
}
