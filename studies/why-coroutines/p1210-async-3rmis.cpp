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

class Class01
{
public:
    void function1(int in1, int in2, int testval)
    {
        printf("Class01::function1(in1 = %d, in2 = %d, testval = %d)\n", in1, in2, testval);
        remoteObj1.sendc_op1(in1, in2, 
            [this, in1, in2, testval](int out1, int out2, int ret1) {
                this->function1a(in1, in2, testval, out1, out2, ret1);
            });
        // 1a Do stuff that doesn't need the result of the RMI
    }

    void function1a(int in1, int in2, int testval, int out1, int out2, int ret1)
    {
        printf("Class01::function1a(out1 = %d, out2 = %d, ret1 = %d, testval = %d)\n", out1, out2, ret1, testval);
        // 1b Do stuff that needs the result of the RMI
        if (ret1 == testval) {
            remoteObj2.sendc_op2(in1, in2,
                [this](int out1, int ret1) {
                    this->function1b(out1, ret1);
                });
            // 2a Do stuff that doesn't need the result of the RMI
        }
        else {
            remoteObj3.sendc_op3(in1,
                [this](int out1, int out2, int ret1) {
                    this->function1c(out1, out2, ret1);
                });
            // 3a Do stuff that doesn't need the result of the RMI
        }
    }

    void function1b(int out3, int ret2)
    {
        printf("Class01::function1b(out3 = %d, ret2 = %d)\n", out3, ret2);
        // 2b Do stuff that needs the result of the RMI
    }

    void function1c(int out4, int out5, int ret3)
    {
        printf("Class01::function1c(out4 = %d, out5 = %d, ret3 = %d)\n", out4, out5, ret3);
        // 3b Do stuff that needs the result of the RMI
    }

    /**
     * @brief alternative version of function1 that avoids the introduction of callback functions
     * by placing the original code in lambdas.
     *
     */
    void function1alt(int in1, int in2, int testval)
    {
        printf("Class01::function1alt(in1 = %d, in2 = %d, testval = %d)\n", in1, in2, testval);
        remoteObj1.sendc_op1(in1, in2, 
            [this, in1, in2, testval](int out1, int out2, int ret1)
            { 
                printf("Class01::function1alt: 1: out1 = %d, out2 = %d, ret1 = %d\n", out1, out2, ret1);
                // 1b Do stuff that needs the result of the RMI
                if (ret1 == testval) {
                    remoteObj2.sendc_op2(in1, in2,
                        [this](int out3, int ret2)
                        {
                            printf("Class01::function1alt: 2: out3 = %d, ret2 = %d\n", out3, ret2);
                            // 2b Do stuff that needs the result of the RMI
                        });
                    // 2a Do stuff that doesn't need the result of the RMI
                }
                else {
                    remoteObj3.sendc_op3(in1,
                        [this](int out4, int out5, int ret3) 
                        {
                            printf("Class01::function1alt: 3: out4 = %d, out5 = %d, ret3 = %d\n", out4, out5, ret3);
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
#if 0
    class01.function1(11, 12, 10);
    eventQueue.run();
    printf("\n");
    class01.function1(11, 12, 23);
    eventQueue.run();
    printf("\n");
    class01.function1alt(11, 12, 10);
    eventQueue.run();
    printf("\n");
    class01.function1alt(11, 12, 23);
    eventQueue.run();
    printf("\n");
#else
    class01.function1(11, 12, 10);
    class01.function1(11, 12, 23);
    class01.function1alt(11, 12, 10);
    class01.function1alt(11, 12, 23);
    printf("\n");
    eventQueue.run();
#endif
    return 0;
}
