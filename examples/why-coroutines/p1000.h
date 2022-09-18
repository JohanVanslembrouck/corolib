/**
 * @file p1000.h
 * @brief Contains a RemoteObject1 class that is common to all p1000-* variants.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1000_H_
#define _P1000_H_

#include "common.h"

class RemoteObject1
{
public:
    int op1(int in11, int in12, int& out11, int& out12)
    {
        printf("RemoteObject1::op1(%d, %d, %d, %d)\n", in11, in12, out11, out12);
        // Marshal in11 and in12 into a buffer to write to the remote object.
        // Write the buffer to the remote object.
        // Read the response from the remote object.
        // There isn't an remote object or I/O system that will send a response. 
        // Just sleep a while to simulate waiting for the response.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // The reponse has arrived.
        // Unmarshal out12 and out12 from the response buffer.
        out11 = 1;
        out12 = 2;
        return 3; 
    }
    
    void sendc_op1(int in11, int in12, lambda_3int_t lambda)
    {
        printf("RemoteObject1::sendc_op1(%d, %d, lambda)\n", in11, in12);
        // Marshal in11 and in12 into a buffer to write to the remote object.
        // Write the buffer to the remote object.
        
        // There isn't an I/O system that will place the lambda in the event queue
        // when an I/O event arrives. Therefore we do it ourselves.
        eventQueue.push([lambda]() { lambda(1, 2, 3); });
    }
};

#endif
