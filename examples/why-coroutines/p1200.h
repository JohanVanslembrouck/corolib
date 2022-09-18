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
    int op1(int in11, int in12, int& out11, int& out12)
    {
        printf("RemoteObject1::op1(%d, %d, %d, %d)\n", in11, in12, out11, out12);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return 0; 
    }

    int op2(int in21, int in22, int& out21)
    {
        printf("RemoteObject1::op2(%d, %d, %d)\n", in21, in22, out21);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return 0;
    }
  
    int op3(int in31, int& out31, int& out32)
    {
        printf("RemoteObject1::op3(%d, %d, %d)\n", in31, out31, out32);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return 0;
    }
    
    void sendc_op1(int in11, int in12, lambda_3int_t lambda)
    {
        printf("RemoteObject1::sendc_op1(%d, %d, l)\n", in11, in12);
        eventQueue.push([lambda]() { lambda(1, 2, 3); });
    }

    void sendc_op2(int in11, int in12, lambda_2int_t lambda)
    {
        printf("RemoteObject1::sendc_op2(%d, %d, l)\n", in11, in12);
        eventQueue.push([lambda]() { lambda(1, 2); });
    }

    void sendc_op3(int in11, lambda_3int_t lambda)
    {
        printf("RemoteObject1::sendc_op3(%d, l)\n", in11);
        eventQueue.push([lambda]() { lambda(1, 2, 3); });
    }
};

#endif
