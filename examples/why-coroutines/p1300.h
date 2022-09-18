/**
 * @file p1300.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1300_H_
#define _P1300_H_

#include "common.h"

class RemoteObject1
{
public:
    int op1(Msg& msg)
    {
        printf("RemoteObject1::op1(msg)\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return 0;
    }
    
    void sendc_op1(Msg& msg, lambda_void_t lambda)
    {
        printf("RemoteObject1::sendc_op1(msg, lambda)\n");
        eventQueue.push(lambda);
    }
};

#endif
