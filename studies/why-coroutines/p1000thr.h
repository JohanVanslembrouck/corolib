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
        printf("op1(): start_op1(in1, in2);\n");
        int ret;
        start_op1(in1, in2, out1, out2, ret);
        printf("op1(): await();\n");
        await();
        return ret;
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
