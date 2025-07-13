/**
 * @file p1000thr.cpp
 * @brief 
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */
 
#include "p1000thr.h"

void RemoteObject1Thr::start_op1(int in1, int in2, int& _out1, int& _out2, int& _ret1)
{
    printf("RemoteObject1Thr::start_op1(%d, %d)\n", in1, in2);
    m_remoteObject.startthr_op1(in1, in2,
        [this, &_out1, &_out2, &_ret1](int out1, int out2, int ret1)
        {
            printf("RemoteObject1Thr completion handler: out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
            _out1 = out1;
            _out2 = out2;
            _ret1 = ret1;
            m_binsema.release();
        });
}