/**
 * @file p1300.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#ifndef _P1300_H_
#define _P1300_H_

#include "common.h"

class RemoteObject1
{
public:
    int op1(Msg& msg)
    {
        printf("RemoteObject1::op1(msg): msg.m_length = %d\n", msg.m_length);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return 0;
    }
    
    void sendc_op1(Msg& msg, lambda_void_t lambda)
    {
        printf("RemoteObject1::sendc_op1(msg, lambda): msg.m_length = %d\n", msg.m_length);
		registerCB(lambda);
    }

    void startthr_op1(Msg& msg, lambda_void_t lambda)
    {
        printf("RemoteObject1::startthr_op1(msg, lambda)\n");
        startThread(lambda);
    }
};

#endif
