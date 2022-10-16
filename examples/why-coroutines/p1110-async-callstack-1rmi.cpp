/**
 * @file p1110-async-callstack-1rmi.cpp
 * @brief Asynchronous implementation of p1100-sync-callstack-1rmi.cpp.
 *
 * Every layer class (except the highest layer) has a data member to store the lambda passed
 * by the higher layer when calling function1 of that layer.
 * This lambda is then used to callback from the lower layer into the higher layer.
 *
 * This simple implementation has a severe problem:
 * when the application calls a second function while the first one 
 * has not yet returned its result to the application,
 * the lambda data member will be overwritten with the lambda associated to the second function.
 *
 * This problem is illustrated with the application calling layer03.function1,
 * immediately followed by calling layer03.function2.
 * The consequence is that Layer03::function2_cb will be called twice, 
 * while Layer03::function1_cb will not be called.
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"

#include "p1000.h"

RemoteObject1 remoteObj1;

/**
 * @brief Layer01 is the lowest level in the application stack
 * Lower layer: RemoteObject1
 * Upper layer: Layer02 (via m_lambda)
 */
class Layer01
{
public:
    // Synchronous function:
    // int function1(int in11, int& out12, int& out12)
    
    void function1(int in1, lambda_2int_t lambda) 
    {
        printf("Layer01::function1(): part 1\n");
        m_lambda = lambda;
        remoteObj1.sendc_op1(in1, in1, 
            [this](int out1, int out2, int ret1)
            { 
                this->function1_cb(out1, out2, ret1); 
            });
    }

    void function1_cb(int out11, int out12, int ret1) 
    {
        printf("Layer01::function1_cb(%d, %d, %d)\n", out11, out12, ret1);
        printf("Layer01::function1_cb(): part 2\n");
        // call function1_cb of upper layer
        m_lambda(out11, ret1);
    }
private:
    lambda_2int_t m_lambda;
};

Layer01 layer01;

/**
 * @brief Layer02 is the middle layer in the application stack
 * Lower layer: Layer01
 * Upper layer: Layer03 (via m_lambda)
 */
class Layer02
{
public:
    // Synchronous function:
    // int function1(int in1, int& out11)
    
    void function1(int in1, lambda_1int_t lambda)
    {
        printf("Layer02::function1(): part 1\n");
        m_lambda = lambda;
        layer01.function1(in1, 
            [this](int out1, int ret1) { 
                this->function1_cb(out1, ret1);
            });
    }

    void function1_cb(int out11, int ret1)
    {
        printf("Layer02::function1_cb(%d, %d)\n", out11, ret1);
        printf("Layer02::function1_cb(): part 2\n");
        // call function1_cb of upper layer
        m_lambda(ret1);
    }
private:
    lambda_1int_t m_lambda;
};

Layer02 layer02;

/**
 * @brief Layer03 is the upper layer in the application stack
 * Lower layer: Layer02
 * Upper layer: application (not known by Layer03)
 *
 */
class Layer03
{
public:
    // Synchronous function:
    // int function1(int in1);

    void function1(int in1)
    {
        printf("Layer03::function1(): part 1\n");
        layer02.function1(in1,
            [this](int ret1) { 
                this->function1_cb(ret1); 
            });
    }

    void function1_cb(int ret1)
    {
        printf("Layer03::function1_cb(%d)\n", ret1);
        printf("Layer03::function1_cb(): part 2\n");
    }

    // Synchronous function:
    // int function1(int in1);
    
    void function2(int in1)
    {
        printf("Layer03::function2(): part 1\n");
        layer02.function1(in1,
            [this](int ret1) {
                this->function2_cb(ret1);
            });
    }

    void function2_cb(int ret1)
    {
        printf("Layer03::function2_cb(%d)\n", ret1);
        printf("Layer03::function2_cb(): part 2\n");
    }
};

Layer03 layer03;

EventQueue eventQueue;

int main() {
    printf("main();\n");
    eventQueue.push([]() { layer03.function1(2); });
    eventQueue.push([]() { layer03.function2(3); });
    eventQueue.run();
    return 0;
}
