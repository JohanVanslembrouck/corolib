/**
 * @file p1212-async-3rmis.cpp
 * @brief
 *
 * @author Johan Vanslembrouck (johan.vanslembrouck@capgemini.com, johan.vanslembrouck@gmail.com)
 */

#include <stdio.h>

#include "common.h"
#include "variables.h"
#include "eventqueue.h"

#include "p1200.h"

RemoteObject1 remoteObj1;
RemoteObject1 remoteObj2;
RemoteObject1 remoteObj3;

class Class01
{
public:
    void function1()
    {
        printf("Class01::function1()\n");
        remoteObj1.sendc_op1(gin11, gin12, 
            [this](int out1, int out2, int ret1) { this->function1a(out1, out2, ret1); });
        // 1a Do stuff that doesn't need the result of the RMI
    }

    void function1a(int out11, int out12, int ret1)
    {
        printf("Class01::function1a(%d, %d, %d)\n", out11, out12, ret1);
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

    void function1b(int out21, int ret2)
    {
        printf("Class01::function1b(%d, %d)\n", out21, ret2);
        // 2b Do stuff that needs the result of the RMI
    }

    void function1c(int out31, int out32, int ret3)
    {
        printf("Class01::function1c(%d, %d, %d)\n", out31, out32, ret3);
        // 3b Do stuff that needs the result of the RMI
    }
    
    /**
     * @brief alternative version of function1 that avoids the introduction of callback functions
     * by placing the original code in lambdas in lambdas.
     *
     */
    void function1alt()
    {
        printf("Class01::function1()\n");
        remoteObj1.sendc_op1(gin11, gin12, 
            [this](int out1, int out2, int ret1)
            { 
                printf("Class01::function1alt(%d, %d, %d)\n", out1, out2, ret1);
                // 1b Do stuff that needs the result of the RMI
                if (ret1 == gval1) {
                    remoteObj2.sendc_op2(gin21, gin22,
                        [this](int out1, int ret1)
                        {
                            printf("Class01::function1alt(%d, %d)\n", out1, ret1);
                            // 2b Do stuff that needs the result of the RMI 
                        });
                    // 2a Do stuff that doesn't need the result of the RMI
                }
                else {
                    remoteObj3.sendc_op3(gin31, 
                        [this](int out1, int out2, int ret1) 
                        {
                            printf("Class01::function1alt(%d, %d, %d)\n", out1, out2, ret1);
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
    eventQueue.push([]() { class01.function1(); });
    eventQueue.push([]() { class01.function1alt(); });
    eventQueue.run();
    return 0;
}
