/**
 * @file p1212-async-3rmis.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "eventqueue.h"
#include "p1200.h"

RemoteObject1 remoteObj1;
RemoteObject1 remoteObj2;
RemoteObject1 remoteObj3;

int gval1 = 0;
int gret1 = 1, gin11 = 11, gin12 = 12, gout11 = 11, gout12 = 12;
int gret2 = 2, gin21 = 21, gin22 = 22, gout21 = 21;
int gret3 = 3, gin31 = 31, gout31 = 31, gout32 = 32;

class Class01
{
public:
    void function1(int in1, int in2)
    {
        printf("Class01::function1(in1 = %d, in2 = %d)\n", in1, in2);
        remoteObj1.sendc_op1(in1, in2, 
            [this](int out1, int out2, int ret1) { this->function1a(out1, out2, ret1); });
        // 1a Do stuff that doesn't need the result of the RMI
    }

    void function1a(int out1, int out2, int ret1)
    {
        printf("Class01::function1a(out1 = %d, out2 = %d, ret1 = %d)\n", out1, out2, ret1);
        // 1b Do stuff that needs the result of the RMI
        if (ret1 == gval1) {
            remoteObj2.sendc_op2(gin21, gin22,
                [this](int out1, int ret1){ this->function1b(out1, ret1); });
            // 2a Do stuff that doesn't need the result of the RMI
        }
        else {
            remoteObj3.sendc_op3(gin31, 
                [this](int out1, int out2, int ret1) { this->function1c(out1, out2, ret1); });
            // 3a Do stuff that doesn't need the result of the RMI
        }
    }

    void function1b(int out1, int ret2)
    {
        printf("Class01::function1b(out1 = %d, ret2 = %d)\n", out1, ret2);
        // 2b Do stuff that needs the result of the RMI
    }

    void function1c(int out1, int out2, int ret3)
    {
        printf("Class01::function1c(out1 = %d, out2 = %d, ret3 = %d)\n", out1, out2, ret3);
        // 3b Do stuff that needs the result of the RMI
    }
    
    /**
     * @brief alternative version of function1 that avoids the introduction of callback functions
     * by placing the original code in lambdas.
     *
     */
    void function1alt(int in1, int in2)
    {
        printf("Class01::function1alt(in1 = %d, in2 = %d)\n", in1, in2);
        remoteObj1.sendc_op1(in1, in2, 
            [this](int out1, int out2, int ret1)
            { 
                printf("Class01::function1alt(out1 = %d, out2 = %d, ret1 = %d)\n", out1, out2, ret1);
                // 1b Do stuff that needs the result of the RMI
                if (ret1 == gval1) {
                    remoteObj2.sendc_op2(gin21, gin22,
                        [this](int out1, int ret1)
                        {
                            printf("Class01::function1alt(out1 = %d, ret1 = %d)\n", out1, ret1);
                            // 2b Do stuff that needs the result of the RMI 
                        });
                    // 2a Do stuff that doesn't need the result of the RMI
                }
                else {
                    remoteObj3.sendc_op3(gin31, 
                        [this](int out1, int out2, int ret1) 
                        {
                            printf("Class01::function1alt(out1 = %d, out2 = %d, ret1 = %d)\n", out1, out2, ret1);
                            // 3b Do stuff that needs the result of the RMI
                        });
                    // 3a Do stuff that doesn't need the result of the RMI
                }
            });
        // 1a Do stuff that doesn't need the result of the RMI
    }
};

Class01 class01;

EventQueue eventQueue;

int main()
{
    printf("main();\n");
    class01.function1(11, 12);
    class01.function1alt(21, 22);
    eventQueue.run();
    return 0;
}
