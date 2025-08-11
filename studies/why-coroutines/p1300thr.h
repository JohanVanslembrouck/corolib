/**
 * @file p1300thr.h
 * @brief Coroutine "wrapper" class for RemoteObject1.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _P1300THR_H_
#define _P1300THR_H_

#include <semaphore>

#include "p1300.h"

class RemoteObject1Thr
{
public:
    RemoteObject1Thr(RemoteObject1& remoteObject)
        : m_remoteObject(remoteObject)
    {}
 
    int op1(Msg& msg)
    {
        printf("RemoteObject1Thr::op1()\n");
        m_remoteObject.startthr_op1(msg,
            [this]()
            {
                m_binsema.release();
            });

        printf("RemoteObject1Thr::op1(): await();\n");
        await();
        return 0;
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
