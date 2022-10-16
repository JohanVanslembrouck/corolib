/**
 * @file p1000.h
 * @brief Contains a RemoteObject1 class that is common to all p10XX-* and p11XX-* variants.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#ifndef _P1000_H_
#define _P1000_H_

#include "common.h"

/**
 * @brief RemoteObject1 contains a synchronous and asynchronous operation to communicate
 * with a remote object.
 * It is used by synchronous and asynchronous applications as well as indirectly by a coroutine application.
 *
 */
class RemoteObject1
{
public:
    /**
     * @brief synchronous operation
     * @param in1
     * @param in2
     * @param out1
     * @param out2
     * @return
     */
    int op1(int in1, int in2, int& out1, int& out2)
    {
        printf("RemoteObject1::op1(%d, %d, %d, %d)\n", in1, in2, out1, out2);
        // Marshal in1 and in2 into a buffer to write to the remote object.
        // Write the buffer to the remote object.
        // Read the response from the remote object.
        // There isn't an remote object or I/O system that will send a response. 
        // Just sleep a while to simulate waiting for the response.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // The reponse has arrived.
        // Unmarshal out12 and out12 from the response buffer.
        out1 = 1;
        out2 = 2;
        return 3; 
    }
    
    /**
     * @brief asynchronous operation
     * @param in1
     * @param in2
     * @param lambda
     */
    void sendc_op1(int in1, int in2, lambda_3int_t lambda)
    {
        printf("RemoteObject1::sendc_op1(%d, %d, lambda)\n", in1, in2);
        // Marshal in1 and in2 into a buffer to write to the remote object.
        // Write the buffer to the remote object.
        // (write code is not present.)
        
        // Register the lambda with the "communication framework".
        // The framework will call the lambda when it has received the response.
        registerCB(lambda);
    }
};

#endif
