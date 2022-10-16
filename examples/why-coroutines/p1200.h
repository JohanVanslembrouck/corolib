/**
 * @file p1200.h
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1200_H_
#define _P1200_H_

#include "common.h"

class RemoteObject1
{
public:
    // Synchronous functions
    // ---------------------
    
    int op1(int in1, int in2, int& out1, int& out2)
    {
        printf("RemoteObject1::op1(%d, %d, %d, %d)\n", in1, in2, out1, out2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        out1 = 1;
        out2 = 2;
        return 3;
    }

    int op2(int in1, int in2, int& out1)
    {
        printf("RemoteObject1::op2(%d, %d, %d)\n", in1, in2, out1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        out1 = 1;
        return 2;
    }
  
    int op3(int in1, int& out1, int& out2)
    {
        printf("RemoteObject1::op3(%d, %d, %d)\n", in1, out1, out2);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        out1 = 1;
        out2 = 2;
        return 3;
    }
    
    // Asynchronous functions
    // ----------------------
    
    void sendc_op1(int in1, int in2, lambda_3int_t lambda)
    {
        printf("RemoteObject1::sendc_op1(%d, %d, l)\n", in1, in2);
        registerCB(lambda);
    }

    void sendc_op2(int in1, int in2, lambda_2int_t lambda)
    {
        printf("RemoteObject1::sendc_op2(%d, %d, l)\n", in1, in2);
        registerCB(lambda);
    }

    void sendc_op3(int in1, lambda_3int_t lambda)
    {
        printf("RemoteObject1::sendc_op3(%d, l)\n", in1);
        registerCB(lambda);
    }
};

#endif
