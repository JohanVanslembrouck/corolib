/**
 * @file p1000thr.h
 * @brief Coroutine "wrapper" class for RemoteObject1.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _P1000THR_H_
#define _P1000THR_H_

#include <semaphore>

#include "p1000.h"

class RemoteObject1Thr
{
public:
    RemoteObject1Thr(RemoteObject1& remoteObject)
        : m_remoteObject(remoteObject)
    {}
    
    // User API
    /**
     * @brief op1 has same parameter list as op1 in class RemoteObject1
     * @param in1
     * @param in2
     * @param out2
     * @param out2
     * @return int
     */
    int op1(int in1, int in2, int& out1, int& out2)
    {
        printf("RemoteObject1Thr::op1(): start_op1(in1, in2, ...);\n");
        int ret;
        start_op1(in1, in2, out1, out2, ret);
        printf("RemoteObject1Thr::op1(): await();\n");
        await();
        return ret;
    }

    /**
    * @brief op1a has same parameter list as op1 in class RemoteObject1.
    * op1a is a variant of op1 above that does not use the function start_op1.
    * Instead, op1a includes the implementation of start_op1 "inline".
    * The implementation of op1a makes the relationship between the release
    * and the acquire of the binary semaphore clearer.
    * @param in1
    * @param in2
    * @param out2
    * @param out2
    * @return int
    */
    int op1a(int in1, int in2, int& out1, int& out2)
    {
        printf("RemoteObject1Thr::op1a()\n");
        int ret1;
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

    void await()
    {
        m_binsema.acquire();
    }

protected:
    void start_op1(int in1, int in2, int& out1, int& out2, int& ret1);

private:
    RemoteObject1 m_remoteObject;

    std::binary_semaphore m_binsema{ 0 };
};

#endif
