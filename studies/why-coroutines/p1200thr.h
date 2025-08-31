/**
 * @file p1200thr.h
 * @brief Coroutine "wrapper" class for RemoteObject1.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _P1200THR_H_
#define _P1200THR_H_

#include <semaphore>
#include <latch>

#include "p1200.h"

class RemoteObject1Thr
{
public:
    RemoteObject1Thr(RemoteObject1& remoteObject)
        : m_remoteObject(remoteObject)
    {}

    int op1(int in1, int in2, int& out1, int& out2)
    {
        printf("op1(): start_op1(in1, in2);\n");
        int ret1 = -1;
        m_remoteObject.startthr_op1(in1, in2,
            [this, &out1, &out2, &ret1](int out1a, int out2a, int ret1a)
            {
                printf("RemoteObject1Thr completion handler: out1 = %d, out2 = %d, ret1 = %d\n", out1a, out2a, ret1a);
                out1 = out1a;
                out2 = out2a;
                ret1 = ret1a;
                m_binsema.release();
            });
        printf("RemoteObject1Thr::op1(): await();\n");
        await();
        return ret1;
    }

    void op1(int in1, int in2, int& out1, int& out2, int& ret1, std::latch& latch_)
    {
        printf("op1(): start_op1(in1, in2);\n");
        m_remoteObject.startthr_op1(in1, in2,
            [this, &latch_, &out1, &out2, &ret1](int out1a, int out2a, int ret1a)
            {
                printf("RemoteObject1Thr completion handler: out1 = %d, out2 = %d, ret1 = %d\n", out1a, out2a, ret1a);

                out1 = out1a;
                out2 = out2a;
                ret1 = ret1a;
                latch_.count_down();
            });
    }

    int op2(int in1, int in2, int& out1)
    {
        printf("RemoteObject1Thr::op2(%d, %d, %d)\n", in1, in2, out1);
        int ret1;
        m_remoteObject.startthr_op2(in1, in2,
            [this, &out1, &ret1](int out1a, int ret1a)
            {
                printf("RemoteObject1Thr completion handler: out1 = %d,  ret1 = %d\n", out1a, ret1a);
                out1 = out1a;
                ret1 = ret1a;
                m_binsema.release();
            });
        printf("RemoteObject1Thr::op1(): await();\n");
        await();
        return ret1;
    }

    int op3(int in1, int& out1, int& out2)
    {
        printf("RemoteObject1Thr::op3(%d, %d, %d)\n", in1, out1, out2);
        int ret1;
        m_remoteObject.startthr_op3(in1,
            [this, &out1, &out2, &ret1](int out1a, int out2a, int ret1a)
            {
                printf("RemoteObject1Thr completion handler: out1 = %d, out2 = %d, ret1 = %d\n", out1a, out2a, ret1a);
                out1 = out1a;
                out2 = out2a;
                ret1 = ret1a;
                m_binsema.release();
            });
        printf("RemoteObject1Thr::op1(): await();\n");
        await();
        return ret1;
    }

    void await()
    {
        m_binsema.acquire();
    }

private:
    RemoteObject1 m_remoteObject;

    std::binary_semaphore m_binsema{ 0 };
};

#endif
