/**
 * @file p1200cogthr.cpp
 * @brief This file contains the secondd of three implementations of the functions 
 * start_op1_impl, start_op2_impl and start_op3_impl.
 * This implementation calls startthr_op1, startthr_op2 and startthr_op3.
 * 
 * @author Johan Vanslembrouck
 */

#include "p1200cog.h"

void RemoteObject1Co::start_op1_impl(const int idx, int in1, int in2)
{
    print(PRI1, "RemoteObject1Co::start_op1_impl(%d, %d, Md)\n", idx, in1, in2);

    m_remoteObject.startthr_op1(in1, in2,
        [this, idx](op1_ret_t in1)
        {
            print(PRI1, "%p: RemoteObject1Co::start_op1_impl(): completion handler\n", this);
            genericCompletionHandler<op1_ret_t>(idx, in1);
        });
}

void RemoteObject1Co::start_op2_impl(const int idx, int in1, int in2)
{
    print(PRI1, "RemoteObject1Co::start_op2_impl(%d, %d, %d)\n", idx, in1, in2);

    m_remoteObject.startthr_op2(in1, in2,
        [this, idx](op2_ret_t in1)
        {
            print(PRI1, "%p: RemoteObject1Co::start_op2_impl(): completion handler\n", this);
            genericCompletionHandler<op2_ret_t>(idx, in1);
        });
}

void RemoteObject1Co::start_op3_impl(const int idx, int in1)
{
    print(PRI1, "RemoteObj1::start_op3_impl(%d, %d)\n", idx, in1);

    m_remoteObject.startthr_op3(in1,
        [this, idx](op1_ret_t in1)
        {
            print(PRI1, "%p: RemoteObject1Co::start_op3_impl(): completion handler\n", this);
            genericCompletionHandler<op1_ret_t>(idx, in1);
        });
}
