/**
 * @file p1110-async-callstack-1rmi.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"

// -------------------------------------------------

class RemoteObject1
{
public:
    void sendc_op1(int in11, int in12, lambda_3int_t lambda)
    {
        printf("RemoteObject1::sendc_op1(%d, %d, lambda)\n", in11, in12);
        eventQueue.push([lambda]() { lambda(1, 2, 3); });
    }
};

RemoteObject1 remoteObj1;

// -------------------------------------------------

class Layer01
{
public:
    // int function1(int in11, int& out12, int& out12)
    
    void function1(int in1, lambda_2int_t lambda) 
    {
        printf("Layer01::function1(): part 1\n");
        m_lambda = lambda;
        remoteObj1.sendc_op1(in1, in1, 
            [this](int out1, int out2, int ret1) { 
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

// -------------------------------------------------

class Layer02
{
public:
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

// -------------------------------------------------

class Layer03
{
public:
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
};

Layer03 layer03;

// -------------------------------------------------

int main() {
    printf("main();\n");
    connect(event1, []() { layer03.function1(2); });
    connect(event2, []() { layer03.function1(3); });
    eventQueue.run();
    return 0;
}
